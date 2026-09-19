// ieee_lookup.js — 按标题在 IEEE Xplore 定向检索，拿到 articleNumber / 元数据
// 由 cdpgrab.js eval 调用；标题来自 window.__TITLES（由调用方注入）或内置列表
(async () => {
  const TITLES = window.__TITLES || [];
  const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

  function norm(s) {
    return (s || "").toLowerCase().replace(/[^a-z0-9]+/g, " ").trim();
  }
  function sim(a, b) {
    const wa = new Set(norm(a).split(" ")), wb = new Set(norm(b).split(" "));
    if (!wa.size || !wb.size) return 0;
    let inter = 0;
    wa.forEach((w) => { if (wb.has(w)) inter++; });
    const jac = inter / (wa.size + wb.size - inter);
    const na = norm(a), nb = norm(b);
    let pre = 0;
    for (let i = 0; i < Math.min(na.length, nb.length); i++) { if (na[i] !== nb[i]) break; pre++; }
    return 0.7 * jac + 0.3 * (pre / Math.max(na.length, nb.length));
  }

  async function lookup(title) {
    const body = {
      newsearch: true,
      queryText: '("Document Title":"' + title.replace(/"/g, "") + '")',
      highlight: false,
      returnFacets: ["ALL"],
      returnType: "SEARCH",
      matchPubs: true,
      rowsPerPage: 5,
      pageNumber: 1,
    };
    const r = await fetch("/rest/search", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      credentials: "include",
      body: JSON.stringify(body),
    });
    if (!r.ok) return { error: r.status };
    const j = await r.json();
    const recs = j.records || [];
    if (!recs.length) return { hits: 0 };
    const scored = recs.map((x) => ({ s: sim(title, x.articleTitle), x })).sort((a, b) => b.s - a.s);
    const best = scored[0];
    if (best.s < 0.45) return { hits: recs.length, best: null, near: recs.slice(0, 2).map((x) => x.articleTitle) };
    const x = best.x;
    return {
      hits: recs.length,
      score: +best.s.toFixed(3),
      best: {
        id: x.articleNumber,
        title: x.articleTitle,
        venue: x.publicationTitle,
        year: x.publicationYear,
        date: x.publicationDate,
        cites: x.citationCount,
        downloads: x.downloadCount,
        access: x.accessType && x.accessType.type,
        doi: x.doi,
        pdf: x.pdfLink,
        authors: (x.authors || []).map((a) => a.preferredName),
        abstract: (x.abstract || "").slice(0, 900),
      },
    };
  }

  const out = {};
  let i = 0;
  for (const t of TITLES) {
    i++;
    try {
      out[t] = await lookup(t);
    } catch (e) {
      out[t] = { error: String(e) };
    }
    const b = out[t].best;
    if (b) {
      window.__PROGRESS = i + "/" + TITLES.length;
    }
    await sleep(300);
  }
  return JSON.stringify({ lookedUpAt: new Date().toISOString(), n: TITLES.length, out });
})()
