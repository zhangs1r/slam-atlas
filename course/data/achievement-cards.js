/* ============================================================
   data/achievement-cards.js — 成就馆 · 全息收藏卡数据
   ------------------------------------------------------------
   一张卡对应一个「来源块」(collection)。
   解锁条件：该 collection 下的全部课程都被标记为已完成
   （localStorage 的 paperlesson_done 里包含全部 lesson.id）。
   字段说明：
     id          卡片 id，约定为 "ach-<collectionId>"
     title       卡片名
     icon        卡片图标（emoji）
     paper       来源块名称（显示在卡片右下）
     date        生成日期
     rarity      "legendary" | "common"（决定边框与稀有度圆点）
     thumb_b64   缩略图，直接放 data URI（SVG / PNG 都行）
     card_file   点击后在全息模态里打开的 HTML 文件（相对 course/index.html）
   ============================================================ */

window.ACHIEVEMENT_CARDS = {
  cards: [
    {
      id: "ach-stage-a",
      title: "坐标系建立者",
      icon: "🧭",
      paper: "阶段 A · 先建立坐标系",
      date: "2026-09-16",
      rarity: "legendary",
      thumb_b64:
        "data:image/svg+xml;utf8," +
        "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 400 300'>" +
        "<defs>" +
        "<linearGradient id='bg' x1='0' y1='0' x2='1' y2='1'>" +
        "<stop offset='0' stop-color='rgb(204,120,92)'/>" +
        "<stop offset='0.55' stop-color='rgb(178,98,72)'/>" +
        "<stop offset='1' stop-color='rgb(120,62,44)'/>" +
        "</linearGradient>" +
        "<radialGradient id='glow' cx='0.5' cy='0.35' r='0.6'>" +
        "<stop offset='0' stop-color='rgba(255,240,230,0.55)'/>" +
        "<stop offset='1' stop-color='rgba(255,240,230,0)'/>" +
        "</radialGradient>" +
        "</defs>" +
        "<rect width='400' height='300' fill='url(%23bg)'/>" +
        "<rect width='400' height='300' fill='url(%23glow)'/>" +
        "<circle cx='200' cy='112' r='52' fill='none' stroke='rgba(255,255,255,0.85)' stroke-width='2'/>" +
        "<circle cx='200' cy='112' r='34' fill='none' stroke='rgba(255,255,255,0.45)' stroke-width='1.5'/>" +
        "<circle cx='200' cy='112' r='16' fill='none' stroke='rgba(255,255,255,0.3)' stroke-width='1'/>" +
        "<line x1='200' y1='52' x2='200' y2='172' stroke='rgba(255,255,255,0.55)' stroke-width='1.5'/>" +
        "<line x1='140' y1='112' x2='260' y2='112' stroke='rgba(255,255,255,0.55)' stroke-width='1.5'/>" +
        "<polygon points='200,50 194,66 206,66' fill='rgb(255,244,236)'/>" +
        "<text x='200' y='216' font-size='30' text-anchor='middle' fill='rgb(255,248,244)' font-family='Georgia, serif' font-weight='bold'>坐标系建立者</text>" +
        "<text x='200' y='244' font-size='14' text-anchor='middle' fill='rgba(255,245,240,0.85)' font-family='Helvetica, Arial, sans-serif'>阶段 A · 20 篇综述 · 16 节课</text>" +
        "<text x='200' y='268' font-size='12' text-anchor='middle' fill='rgba(255,245,240,0.6)' font-family='Helvetica, Arial, sans-serif'>LEGENDARY · SLAM PAPER LESSON</text>" +
        "</svg>",
      card_file: "reference/成就卡-第一季.html"
    },
    {
      id: "ach-stage-b",
      title: "地基浇筑者",
      icon: "🧱",
      paper: "阶段 B · 打地基",
      date: "2026-09-17",
      rarity: "legendary",
      thumb_b64:
        "data:image/svg+xml;utf8," +
        "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 400 300'>" +
        "<defs>" +
        "<linearGradient id='bg2' x1='0' y1='0' x2='1' y2='1'>" +
        "<stop offset='0' stop-color='rgb(84,112,127)'/>" +
        "<stop offset='0.55' stop-color='rgb(61,84,97)'/>" +
        "<stop offset='1' stop-color='rgb(37,51,61)'/>" +
        "</linearGradient>" +
        "<radialGradient id='glow2' cx='0.5' cy='0.4' r='0.62'>" +
        "<stop offset='0' stop-color='rgba(240,249,254,0.42)'/>" +
        "<stop offset='1' stop-color='rgba(240,249,254,0)'/>" +
        "</radialGradient>" +
        "</defs>" +
        "<rect width='400' height='300' fill='url(%23bg2)'/>" +
        "<rect width='400' height='300' fill='url(%23glow2)'/>" +
        "<g stroke='rgba(240,249,254,0.85)' stroke-width='1.6' fill='none'>" +
        "<polygon points='100,204 148,228 148,250 100,226' fill='rgba(240,249,254,0.13)'/>" +
        "<polygon points='148,228 196,204 196,226 148,250' fill='rgba(240,249,254,0.20)'/>" +
        "<polygon points='100,204 148,180 196,204 148,228' fill='rgba(240,249,254,0.32)'/>" +
        "<polygon points='152,204 200,228 200,250 152,226' fill='rgba(240,249,254,0.13)'/>" +
        "<polygon points='200,228 248,204 248,226 200,250' fill='rgba(240,249,254,0.20)'/>" +
        "<polygon points='152,204 200,180 248,204 200,228' fill='rgba(240,249,254,0.32)'/>" +
        "<polygon points='126,180 174,204 174,226 126,202' fill='rgba(240,249,254,0.13)'/>" +
        "<polygon points='174,204 222,180 222,202 174,226' fill='rgba(240,249,254,0.20)'/>" +
        "<polygon points='126,180 174,156 222,180 174,204' fill='rgba(240,249,254,0.34)'/>" +
        "<polygon points='150,156 198,180 198,202 150,178' fill='rgba(255,233,201,0.18)'/>" +
        "<polygon points='198,180 246,156 246,178 198,202' fill='rgba(255,233,201,0.28)'/>" +
        "<polygon points='150,156 198,132 246,156 198,180' fill='rgba(255,233,201,0.46)' stroke='rgba(255,236,208,0.95)'/>" +
        "</g>" +
        "<line x1='198' y1='132' x2='198' y2='104' stroke='rgba(240,249,254,0.55)' stroke-width='1.4' stroke-dasharray='5 4'/>" +
        "<g stroke='rgba(240,249,254,0.60)' stroke-width='1.4' fill='none'>" +
        "<line x1='198' y1='104' x2='146' y2='84'/>" +
        "<line x1='198' y1='104' x2='250' y2='84'/>" +
        "<line x1='146' y1='84' x2='198' y2='132'/>" +
        "<line x1='250' y1='84' x2='198' y2='132'/>" +
        "</g>" +
        "<circle cx='198' cy='104' r='6' fill='rgb(249,252,255)'/>" +
        "<circle cx='146' cy='84'  r='4.6' fill='rgb(255,236,208)'/>" +
        "<circle cx='250' cy='84'  r='4.6' fill='rgb(255,236,208)'/>" +
        "<line x1='80' y1='262' x2='320' y2='262' stroke='rgba(240,249,254,0.30)' stroke-width='1.2'/>" +
        "<text x='200' y='292' font-size='15' text-anchor='middle' fill='rgba(240,249,254,0.78)' font-family='Helvetica, Arial, sans-serif'>阶段 B · 19 篇 · 23 节课 · 5 模块</text>" +
        "</svg>",
      card_file: "reference/成就卡-第二季.html"
    },
    {
      id: "ach-stage-c",
      title: "视觉主干驾驶员",
      icon: "📷",
      paper: "阶段 C · 视觉主干",
      date: "2026-09-18",
      rarity: "legendary",
      thumb_b64:
        "data:image/svg+xml;utf8," +
        "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 400 300'>" +
        "<defs>" +
        "<linearGradient id='bgc' x1='0' y1='0' x2='1' y2='1'>" +
        "<stop offset='0' stop-color='rgb(206,124,88)'/>" +
        "<stop offset='0.5' stop-color='rgb(168,94,72)'/>" +
        "<stop offset='1' stop-color='rgb(92,58,64)'/>" +
        "</linearGradient>" +
        "<radialGradient id='glowc' cx='0.5' cy='0.32' r='0.62'>" +
        "<stop offset='0' stop-color='rgba(255,242,232,0.5)'/>" +
        "<stop offset='1' stop-color='rgba(255,242,232,0)'/>" +
        "</radialGradient>" +
        "</defs>" +
        "<rect width='400' height='300' fill='url(%23bgc)'/>" +
        "<rect width='400' height='300' fill='url(%23glowc)'/>" +
        // 房间：地板 + 后墙 + 墙角线
        "<polygon points='54,206 346,206 314,116 86,116' fill='rgba(255,244,236,0.10)'/>" +
        "<polygon points='86,116 314,116 300,52 100,52' fill='rgba(255,244,236,0.07)'/>" +
        "<line x1='86' y1='116' x2='314' y2='116' stroke='rgba(255,248,242,0.45)' stroke-width='1.6'/>" +
        "<line x1='54' y1='206' x2='346' y2='206' stroke='rgba(255,248,242,0.30)' stroke-width='1.4'/>" +
        // 房间里的两件真实家具
        "<rect x='112' y='150' width='52' height='40' rx='4' fill='rgba(255,238,220,0.28)' stroke='rgba(255,248,242,0.55)'/>" +
        "<rect x='252' y='138' width='44' height='52' rx='4' fill='rgba(255,238,220,0.22)' stroke='rgba(255,248,242,0.5)'/>" +
        // 相机轨迹（曲线）
        "<path d='M60,190 C130,168 186,142 250,132 C300,124 336,140 356,166' fill='none' stroke='rgba(255,246,238,0.85)' stroke-width='2.6' stroke-dasharray='7,5'/>" +
        // 轨迹上的三台相机
        "<g fill='rgb(255,247,240)' stroke='rgb(126,72,52)' stroke-width='1.4'>" +
        "<rect x='76' y='174' width='22' height='15' rx='3'/>" +
        "<rect x='180' y='140' width='22' height='15' rx='3'/>" +
        "<rect x='288' y='128' width='22' height='15' rx='3'/>" +
        "</g>" +
        // 相机镜头
        "<circle cx='97' cy='181' r='4.4' fill='none' stroke='rgb(126,72,52)' stroke-width='1.6'/>" +
        "<circle cx='201' cy='147' r='4.4' fill='none' stroke='rgb(126,72,52)' stroke-width='1.6'/>" +
        "<circle cx='309' cy='135' r='4.4' fill='none' stroke='rgb(126,72,52)' stroke-width='1.6'/>" +
        // 视锥
        "<polygon points='97,181 150,140 150,214' fill='rgba(255,246,238,0.16)'/>" +
        "<polygon points='309,135 356,142 350,196' fill='rgba(255,246,238,0.14)'/>" +
        // 地图点（稀疏 + 稠密两种，象征两条路线）
        "<g fill='rgb(255,240,214)'>" +
        "<circle cx='128' cy='126' r='2.6'/><circle cx='164' cy='118' r='2.6'/>" +
        "<circle cx='206' cy='106' r='2.6'/><circle cx='244' cy='112' r='2.6'/>" +
        "<circle cx='286' cy='102' r='2.6'/>" +
        "</g>" +
        "<g fill='none' stroke='rgba(255,236,206,0.55)' stroke-width='1'>" +
        "<circle cx='128' cy='126' r='5'/><circle cx='206' cy='106' r='5'/><circle cx='286' cy='102' r='5'/>" +
        "</g>" +
        "<line x1='80' y1='262' x2='320' y2='262' stroke='rgba(240,249,254,0.30)' stroke-width='1.2'/>" +
        "<text x='200' y='292' font-size='15' text-anchor='middle' fill='rgba(240,249,254,0.78)' font-family='Helvetica, Arial, sans-serif'>阶段 C · 42 篇 · 视觉主干 · 7 模块</text>" +
        "</svg>",
      card_file: "reference/成就卡-第三季.html"
    },
    {
      id: "ach-stage-d",
      title: "激光雷达手",
      icon: "📡",
      paper: "阶段 D · 激光主干",
      date: "2026-09-19",
      rarity: "legendary",
      thumb_b64:
        "data:image/svg+xml;utf8," +
        "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 400 300'>" +
        "<defs>" +
        "<linearGradient id='bgd' x1='0' y1='0' x2='1' y2='1'>" +
        "<stop offset='0%' stop-color='#1b3a2f'/><stop offset='55%' stop-color='#24503f'/><stop offset='100%' stop-color='#0f2b22'/>" +
        "</linearGradient>" +
        "<radialGradient id='glowd' cx='0.5' cy='0.28' r='0.72'>" +
        "<stop offset='0%' stop-color='#5fd39a' stop-opacity='0.42'/><stop offset='100%' stop-color='#5fd39a' stop-opacity='0'/>" +
        "</radialGradient>" +
        "</defs>" +
        "<rect width='400' height='300' rx='18' fill='url(#bgd)'/>" +
        "<rect width='400' height='300' rx='18' fill='url(#glowd)'/>" +
        "<circle cx='200' cy='78' r='58' fill='none' stroke='#5fd39a' stroke-opacity='0.30' stroke-width='1.2'/>" +
        "<circle cx='200' cy='78' r='40' fill='none' stroke='#5fd39a' stroke-opacity='0.45' stroke-width='1.2'/>" +
        "<circle cx='200' cy='78' r='22' fill='none' stroke='#5fd39a' stroke-opacity='0.62' stroke-width='1.2'/>" +
        "<rect x='172' y='70' width='56' height='16' rx='8' fill='#8ee8b8'/>" +
        "<rect x='188' y='56' width='24' height='16' rx='4' fill='#d8fff0'/>" +
        "<line x1='200' y1='78' x2='284' y2='150' stroke='#5fd39a' stroke-width='1.6' stroke-opacity='0.62'/>" +
        "<line x1='200' y1='78' x2='116' y2='150' stroke='#5fd39a' stroke-width='1.6' stroke-opacity='0.62'/>" +
        "<circle cx='284' cy='150' r='3.6' fill='#bff5d8'/><circle cx='116' cy='150' r='3.6' fill='#bff5d8'/>" +
        "<circle cx='252' cy='176' r='3' fill='#8ee8b8'/><circle cx='148' cy='176' r='3' fill='#8ee8b8'/>" +
        "<text x='200' y='205' font-size='30' text-anchor='middle' fill='#ffffff' font-family='Helvetica, Arial, sans-serif'>📡</text>" +
        "<text x='200' y='242' font-size='27' font-weight='700' text-anchor='middle' fill='#ffffff' font-family='Helvetica, Arial, sans-serif'>激光雷达手</text>" +
        "<text x='200' y='270' font-size='14' text-anchor='middle' fill='rgba(216,255,240,0.82)' font-family='Helvetica, Arial, sans-serif'>从 LOAM 到 FAST-LIO2 · 把 ikd-Tree 的债还了</text>" +
        "<text x='200' y='291' font-size='15' text-anchor='middle' fill='rgba(216,255,240,0.78)' font-family='Helvetica, Arial, sans-serif'>阶段 D · 32 篇 · 激光主干 · 8 模块</text>" +
        "</svg>",
      card_file: "reference/成就卡-第四季.html"
    }
  ]
};
