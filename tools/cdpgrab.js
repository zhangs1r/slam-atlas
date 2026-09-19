// cdpgrab.js — 通过 CDP 驱动独立 Chrome 抓取需要浏览器环境才能下载的资源（如 IEEE Xplore PDF）
// 依赖：Node >= 21（内置 WebSocket / fetch）
//
// 用法:
//   node cdpgrab.js ensure
//   node cdpgrab.js get <pageUrl> <pdfUrl> <outFile>
//   node cdpgrab.js eval <pageUrl> <jsFile>          # 在页面里跑 JS，返回 JSON 字符串
//   node cdpgrab.js text <pageUrl> <outFile>         # 抓取页面纯文本
const { spawn } = require('child_process');
const fs = require('fs');
const path = require('path');

const CHROME = process.env.CHROME_BIN || 'C:/Program Files/Google/Chrome/Application/chrome.exe';
const PORT = parseInt(process.env.CDP_PORT || '9222', 10);
const PROFILE = process.env.CDP_PROFILE || 'G:/project/ieeexplore/.cdp-profile';
const HOST = '127.0.0.1';
const CDP_TIMEOUT = parseInt(process.env.CDP_TIMEOUT || '120000', 10);

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

async function httpJson(pathname, method = 'GET') {
  const res = await fetch(`http://${HOST}:${PORT}${pathname}`, { method });
  if (!res.ok) throw new Error(`HTTP ${res.status} for ${pathname}`);
  const txt = await res.text();
  try { return JSON.parse(txt); } catch (e) { return txt; }
}

async function isUp() {
  try {
    const v = await httpJson('/json/version');
    return v && v.webSocketDebuggerUrl ? v : null;
  } catch (e) { return null; }
}

async function ensureChrome() {
  let v = await isUp();
  if (v) { log(`chrome already up: ${v.Browser}`); return v; }
  if (!fs.existsSync(CHROME)) throw new Error(`chrome not found at ${CHROME}`);
  fs.mkdirSync(PROFILE, { recursive: true });
  const args = [
    `--remote-debugging-port=${PORT}`,
    `--user-data-dir=${PROFILE}`,
    '--remote-allow-origins=*',
    '--no-first-run',
    '--no-default-browser-check',
    '--disable-session-crashed-bubble',
    '--disable-infobars',
    '--hide-crash-restore-bubble',
    'about:blank',
  ];
  log(`launching chrome: ${CHROME}`);
  const p = spawn(CHROME, args, { detached: true, stdio: 'ignore' });
  p.unref();
  for (let i = 0; i < 60; i++) {
    await sleep(500);
    v = await isUp();
    if (v) { log(`chrome up after ${(i + 1) * 0.5}s: ${v.Browser}`); return v; }
  }
  throw new Error('chrome did not expose CDP in time');
}

function log(m) { process.stdout.write(`[cdpgrab] ${m}\n`); }

// ---- minimal CDP client ----
class CDP {
  constructor(wsUrl) {
    this.wsUrl = wsUrl;
    this.id = 0;
    this.pending = new Map();
    this.events = [];
    this.listeners = [];
  }
  async connect() {
    this.ws = new WebSocket(this.wsUrl);
    await new Promise((res, rej) => {
      this.ws.addEventListener('open', res, { once: true });
      this.ws.addEventListener('error', (e) => rej(new Error('ws error')), { once: true });
    });
    this.ws.addEventListener('message', (ev) => {
      let msg;
      try { msg = JSON.parse(ev.data); } catch (e) { return; }
      if (msg.id && this.pending.has(msg.id)) {
        const { resolve, reject } = this.pending.get(msg.id);
        this.pending.delete(msg.id);
        if (msg.error) reject(new Error(`${msg.error.code}: ${msg.error.message}`));
        else resolve(msg.result);
      } else if (msg.method) {
        this.events.push(msg);
        this.listeners.forEach((fn) => fn(msg));
      }
    });
    return this;
  }
  send(method, params = {}) {
    const id = ++this.id;
    return new Promise((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      this.ws.send(JSON.stringify({ id, method, params }));
      setTimeout(() => {
        if (this.pending.has(id)) {
          this.pending.delete(id);
          reject(new Error(`timeout: ${method}`));
        }
      }, CDP_TIMEOUT);
    });
  }
  waitEvent(method, timeout = 45000) {
    return new Promise((resolve, reject) => {
      const hit = this.events.find((e) => e.method === method);
      if (hit) return resolve(hit);
      const t = setTimeout(() => reject(new Error(`timeout waiting ${method}`)), timeout);
      const fn = (msg) => {
        if (msg.method === method) {
          clearTimeout(t);
          this.listeners = this.listeners.filter((f) => f !== fn);
          resolve(msg);
        }
      };
      this.listeners.push(fn);
    });
  }
  close() { try { this.ws.close(); } catch (e) {} }
}

async function newTarget(url) {
  return httpJson(`/json/new?${encodeURIComponent(url)}`, 'PUT');
}
async function closeTarget(id) {
  try { await httpJson(`/json/close/${id}`); } catch (e) {}
}

async function withPage(pageUrl, fn) {
  const t = await newTarget('about:blank');
  const cdp = await new CDP(t.webSocketDebuggerUrl).connect();
  try {
    await cdp.send('Page.enable');
    await cdp.send('Runtime.enable');
    const nav = cdp.send('Page.navigate', { url: pageUrl });
    await nav.catch(() => {});
    await cdp.waitEvent('Page.loadEventFired', 45000).catch(() => {});
    await sleep(1200);
    return await fn(cdp);
  } finally {
    cdp.close();
    await closeTarget(t.id);
  }
}

async function evalJson(cdp, expr, awaitPromise = true) {
  const r = await cdp.send('Runtime.evaluate', {
    expression: expr, awaitPromise, returnByValue: true,
  });
  if (r.exceptionDetails) throw new Error('page eval threw: ' + JSON.stringify(r.exceptionDetails).slice(0, 400));
  return r.result.value;
}

const B64_CHUNK = 512 * 1024;
function b64ChunkExpr(start, size) {
  return `(()=>{const u=window.__pdf;const s=${start};const e=Math.min(u.length,s+${size});let o='';const SB=0x8000;for(let i=s;i<e;i+=SB){o+=String.fromCharCode.apply(null,u.subarray(i,Math.min(i+SB,e)));}return btoa(o);})()`;
}

async function grabPdf(cdp, pdfUrl, outFile) {
  const url = JSON.stringify(pdfUrl);
  // 优先走 Network.loadNetworkResource（不依赖页面上下文，最轻）
  try {
    await cdp.send('Network.enable');
    const r = await cdp.send('Network.loadNetworkResource', {
      url: pdfUrl, options: { disableCache: false, includeCredentials: true },
    });
    const res = r && r.resource;
    if (res && res.success && res.stream) {
      const parts = [];
      let off = 0;
      for (;;) {
        const chunk = await cdp.send('IO.read', { handle: res.stream, offset: off, size: B64_CHUNK });
        if (chunk.data) {
          const buf = Buffer.from(chunk.data, chunk.base64Encoded ? 'base64' : 'utf8');
          parts.push(buf);
          off += buf.length;
        }
        if (chunk.eof) break;
        if (!chunk.data && !chunk.eof) break;
      }
      const body = Buffer.concat(parts);
      if (body.slice(0, 5).toString('latin1') === '%PDF-') {
        fs.writeFileSync(outFile, body);
        return { mode: 'loadNetworkResource', bytes: body.length, status: res.httpStatusCode };
      }
      log(`loadNetworkResource returned non-PDF (${body.length}B, magic=${body.slice(0, 20).toString('latin1').replace(/\n/g, ' ')})`);
    }
  } catch (e) {
    log(`loadNetworkResource failed: ${e.message}`);
  }
  // 回退：页面上下文 fetch（同源，规避 CORS）
  const js = `(async()=>{const r=await fetch(${url},{credentials:'include'});if(!r.ok)return JSON.stringify({ok:false,status:r.status});const b=await r.arrayBuffer();window.__pdf=new Uint8Array(b);return JSON.stringify({ok:true,size:b.byteLength,type:r.headers.get('content-type')});})()`;
  const out = await evalJson(cdp, js);
  const j = typeof out === 'string' ? JSON.parse(out) : out;
  if (!j.ok) throw new Error(`page fetch failed with HTTP ${j.status}`);
  const total = j.size;
  const fd = fs.openSync(outFile, 'w');
  const parts = [];
  for (let off = 0; off < total; off += B64_CHUNK) {
    const b64 = await evalJson(cdp, b64ChunkExpr(off, B64_CHUNK));
    parts.push(Buffer.from(b64, 'base64'));
    if (off % (4 * 1024 * 1024) === 0) log(`  ...${off}/${total}`);
  }
  const body = Buffer.concat(parts);
  fs.writeSync(fd, body);
  fs.closeSync(fd);
  return { mode: 'page-fetch', bytes: body.length, type: j.type };
}

async function main() {
  const [cmd, ...args] = process.argv.slice(2);
  if (cmd === 'ensure') {
    await ensureChrome();
    return;
  }
  if (cmd === 'get') {
    const [pageUrl, pdfUrl, outFile] = args;
    await ensureChrome();
    fs.mkdirSync(path.dirname(outFile), { recursive: true });
    const info = await withPage(pageUrl, (cdp) => grabPdf(cdp, pdfUrl, outFile));
    log(`OK ${outFile} ${JSON.stringify(info)}`);
    return;
  }
  if (cmd === 'batchget') {
    const [manifestPath] = args;
    const items = JSON.parse(fs.readFileSync(manifestPath, 'utf8'));
    await ensureChrome();
    const results = [];
    // 复用同一个页面标签，先导航到任意 ieeexplore 页，再逐条 grab
    const t = await newTarget('about:blank');
    const cdp = await new CDP(t.webSocketDebuggerUrl).connect();
    try {
      await cdp.send('Page.enable');
      await cdp.send('Runtime.enable');
      let lastOrigin = null;
      for (const it of items) {
        const out = it.out;
        fs.mkdirSync(path.dirname(out), { recursive: true });
        if (fs.existsSync(out) && fs.statSync(out).size > 20000) {
          log(`SKIP (exists) ${out}`);
          results.push({ out, skipped: true });
          continue;
        }
        try {
          const origin = new URL(it.pageUrl).origin;
          if (origin !== lastOrigin) {
            await cdp.send('Page.navigate', { url: it.pageUrl });
            await cdp.waitEvent('Page.loadEventFired', 45000).catch(() => {});
            await sleep(1500);
            lastOrigin = origin;
          }
          const info = await grabPdf(cdp, it.pdfUrl, out);
          log(`OK ${it.key || ''} -> ${out} (${info.bytes}B, ${info.mode})`);
          results.push({ key: it.key, out, ok: true, ...info });
        } catch (e) {
          log(`FAIL ${it.key || ''}: ${e.message}`);
          results.push({ key: it.key, out, ok: false, error: e.message });
        }
        await sleep(600);
      }
    } finally {
      cdp.close();
      await closeTarget(t.id);
    }
    const dst = manifestPath.replace(/\.json$/, '') + '.result.json';
    fs.writeFileSync(dst, JSON.stringify(results, null, 1));
    log(`batch done: ${results.filter((r) => r.ok || r.skipped).length}/${results.length} -> ${dst}`);
    return;
  }
  if (cmd === 'show') {
    // 打开一个持久、前置的标签页给用户手动操作（不会被自动关闭）
    const [url] = args;
    await ensureChrome();
    const t = await newTarget(url || 'about:blank');
    const cdp = await new CDP(t.webSocketDebuggerUrl).connect();
    try {
      await cdp.send('Page.enable');
      await sleep(1500);
      await cdp.send('Page.bringToFront').catch(() => {});
      await sleep(500);
      await cdp.send('Page.bringToFront').catch(() => {});
      log(`tab opened & focused: ${t.id} -> ${url}`);
      log('(这个标签页会保持打开，完成操作后告诉我)');
    } finally {
      cdp.close();
    }
    return;
  }
  if (cmd === 'eval') {
    const [pageUrl, jsFile] = args;
    await ensureChrome();
    const js = fs.readFileSync(jsFile, 'utf8');
    const out = await withPage(pageUrl, (cdp) => evalJson(cdp, js));
    process.stdout.write(typeof out === 'string' ? out : JSON.stringify(out, null, 2));
    return;
  }
  if (cmd === 'text') {
    const [pageUrl, outFile] = args;
    await ensureChrome();
    const out = await withPage(pageUrl, (cdp) => evalJson(cdp, 'document.body.innerText'));
    fs.writeFileSync(outFile, out || '');
    log(`text ${(out || '').length} chars -> ${outFile}`);
    return;
  }
  console.error('unknown command. use: ensure | get | eval | text');
  process.exit(1);
}

main().catch((e) => { console.error('[cdpgrab] ERROR', e.message); process.exit(1); });
