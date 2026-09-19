// pdfcatcher.js — 本地 PDF 接收服务
// 浏览器页面通过 fetch POST 把 PDF 字节流发到这里落盘（绕过 IEEE/arXiv 的直连防护）
// 用法: node pdfcatcher.js [port] [outDir]
const http = require('http');
const fs = require('fs');
const path = require('path');

const PORT = parseInt(process.argv[2] || '8899', 10);
const OUT = process.argv[3] || 'G:/project/ieeexplore/papers';
const LOG = 'G:/project/ieeexplore/meta/pdfcatcher.log';

function log(msg) {
  const line = `[${new Date().toISOString()}] ${msg}\n`;
  try { fs.appendFileSync(LOG, line); } catch (e) {}
  process.stdout.write(line);
}

function cors(res) {
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Methods', 'GET,POST,OPTIONS');
  res.setHeader('Access-Control-Allow-Headers', '*');
  res.setHeader('Access-Control-Allow-Private-Network', 'true');
  res.setHeader('Access-Control-Max-Age', '86400');
}

const server = http.createServer((req, res) => {
  const u = new URL(req.url, 'http://127.0.0.1');
  cors(res);

  if (req.method === 'OPTIONS') {
    res.writeHead(204); res.end(); return;
  }

  if (u.pathname === '/ping') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ ok: true, out: OUT, cwd: process.cwd() }));
    return;
  }

  if (u.pathname === '/list') {
    let files = [];
    try {
      files = fs.readdirSync(OUT).map(f => {
        const st = fs.statSync(path.join(OUT, f));
        return { name: f, bytes: st.size, mtime: st.mtime.toISOString() };
      });
    } catch (e) { files = []; }
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ dir: OUT, count: files.length, files }));
    return;
  }

  if (u.pathname === '/save' && req.method === 'POST') {
    const name = (u.searchParams.get('name') || '').replace(/[\\/:*?"<>|]/g, '_');
    if (!name) {
      res.writeHead(400, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ ok: false, error: 'missing name' }));
      return;
    }
    const target = path.join(OUT, name);
    const chunks = [];
    let bytes = 0;
    req.on('data', c => { chunks.push(c); bytes += c.length; });
    req.on('end', () => {
      try {
        fs.writeFileSync(target, Buffer.concat(chunks));
        log(`SAVED ${name} (${bytes} bytes)`);
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ ok: true, name, bytes, path: target }));
      } catch (e) {
        log(`ERROR ${name}: ${e.message}`);
        res.writeHead(500, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ ok: false, error: String(e) }));
      }
    });
    return;
  }

  res.writeHead(404, { 'Content-Type': 'application/json' });
  res.end(JSON.stringify({ ok: false, error: 'not found', path: u.pathname }));
});

server.listen(PORT, '127.0.0.1', () => log(`pdfcatcher listening on 127.0.0.1:${PORT} -> ${OUT}`));
