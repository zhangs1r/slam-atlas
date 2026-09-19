(async () => {
  const QUERIES = [
    "SLAM",
    "visual-inertial SLAM",
    "visual inertial odometry",
    "LiDAR SLAM",
    "LiDAR-inertial odometry",
    "Gaussian splatting SLAM",
    "neural radiance field SLAM",
    "implicit neural representation SLAM",
    "learning-based visual odometry",
    "semantic SLAM",
    "dynamic SLAM",
    "multi-agent SLAM",
    "collaborative SLAM",
    "event-based SLAM",
    "loop closure detection",
    "place recognition",
    "3D Gaussian splatting mapping",
    "SLAM foundation model",
    "deep feature matching SLAM",
    "point cloud registration SLAM",
  ];

  const VENUE_HINTS = [
    "IEEE Transactions on Robotics",
    "IEEE Robotics and Automation Letters",
    "IEEE Transactions on Instrumentation and Measurement",
    "IEEE Robotics & Automation Magazine",
    "IEEE Transactions on Intelligent Transportation Systems",
    "IEEE Transactions on Industrial Electronics",
    "IEEE Transactions on Automation Science and Engineering",
    "IEEE/ASME Transactions on Mechatronics",
    "IEEE Transactions on Aerospace and Electronic Systems",
    "IEEE Transactions on Circuits and Systems for Video Technology",
    "IEEE Transactions on Multimedia",
    "IEEE Transactions on Pattern Analysis and Machine Intelligence",
    "IEEE Journal of Selected Topics in Applied Earth Observations",
    "IEEE Sensors Journal",
    "IEEE Internet of Things Journal",
    "IEEE Transactions on Vehicular Technology",
    "IEEE Transactions on Field Robotics",
    "IEEE Transactions on Cybernetics",
    "IEEE Transactions on Neural Networks and Learning Systems",
    "IEEE Transactions on Geoscience and Remote Sensing",
    "IEEE Transactions on Mobile Computing",
    "IEEE Transactions on Visualization and Computer Graphics",
    "IEEE Transactions on Medical Robotics and Bionics",
    "IEEE International Conference on Robotics and Automation",
    "IEEE/RSJ International Conference on Intelligent Robots and Systems",
    "IEEE/CVF Conference on Computer Vision and Pattern Recognition",
    "IEEE/CVF International Conference on Computer Vision",
    "IEEE International Conference on Intelligent Transportation",
    "International Conference on 3D Vision",
    "IEEE International Conference on Automation Science and Engineering",
    "IEEE Intelligent Vehicles Symposium",
    "IEEE International Conference on Unmanned Aircraft Systems",
    "IEEE International Conference on Industrial Technology",
    "IEEE International Symposium on Safety, Security, and Rescue Robotics",
  ];

  const isTop = (title) => VENUE_HINTS.some((v) => (title || "").includes(v));
  const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

  async function query(q, page) {
    const body = {
      newsearch: true,
      queryText: q,
      highlight: false,
      returnFacets: ["ALL"],
      returnType: "SEARCH",
      matchPubs: true,
      rowsPerPage: 100,
      pageNumber: page,
      ranges: ["2022_2026_Year"],
    };
    const r = await fetch("/rest/search", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      credentials: "include",
      body: JSON.stringify(body),
    });
    if (!r.ok) return { error: r.status, records: [] };
    const j = await r.json();
    return { total: j.totalRecords, records: j.records || [] };
  }

  const out = {};
  const seen = new Set();
  const stats = [];

  for (const q of QUERIES) {
    let kept = 0, tot = 0;
    for (const page of [1, 2, 3]) {
      let res;
      try { res = await query(q, page); } catch (e) { break; }
      if (res.error) break;
      if (page === 1) tot = res.total;
      if (!res.records.length) break;
      for (const rec of res.records) {
        const key = rec.articleNumber;
        if (!key || seen.has(key)) continue;
        if (!isTop(rec.publicationTitle)) continue;
        seen.add(key);
        out[key] = {
          id: key,
          title: rec.articleTitle,
          venue: rec.publicationTitle,
          year: rec.publicationYear,
          date: rec.publicationDate,
          type: rec.contentType,
          doi: rec.doi,
          authors: (rec.authors || []).map((a) => a.preferredName),
          cites: rec.citationCount,
          downloads: rec.downloadCount,
          access: rec.accessType && rec.accessType.type,
          isEarlyAccess: rec.isEarlyAccess,
          pdf: rec.pdfLink,
          abstract: (rec.abstract || "").slice(0, 700),
          queries: [q],
        };
        kept++;
      }
      await sleep(250);
    }
    stats.push({ q, total: tot, kept });
  }

  const records = Object.values(out).sort((a, b) => (b.cites || 0) - (a.cites || 0));
  return JSON.stringify({ harvestedAt: new Date().toISOString(), stats, count: records.length, records });
})()
