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
        "<text x='200' y='268' font-size='12' text-anchor='middle' fill='rgba(255,245,240,0.6)' font-family='Helvetica, Arial, sans-serif'>LEGENDARY · SLAM ATLAS</text>" +
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
    },
    {
      id: "ach-stage-e",
      title: "融合掌舵者",
      icon: "🎛️",
      paper: "阶段 E · 多传感器融合",
      date: "2026-09-19",
      rarity: "legendary",
      thumb_b64:
        "data:image/svg+xml;utf8," +
        "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 400 300'>" +
        "<defs>" +
        "<linearGradient id='bge' x1='0' y1='0' x2='1' y2='1'>" +
        "<stop offset='0%' stop-color='#10333d'/>" +
        "<stop offset='58%' stop-color='#20596a'/>" +
        "<stop offset='100%' stop-color='#0c262e'/>" +
        "</linearGradient>" +
        "<radialGradient id='glowe' cx='50%' cy='40%' r='58%'>" +
        "<stop offset='0%' stop-color='rgba(140,225,240,0.32)'/>" +
        "<stop offset='100%' stop-color='rgba(140,225,240,0)'/>" +
        "</radialGradient>" +
        "</defs>" +
        "<rect width='400' height='300' fill='url(#bge)'/>" +
        "<rect width='400' height='300' fill='url(#glowe)'/>" +
        "<g fill='none' stroke-width='3'>" +
        "<circle cx='152' cy='100' r='48' stroke='#7fd4e4' stroke-opacity='0.9'/>" +
        "<circle cx='248' cy='100' r='48' stroke='#9db8ee' stroke-opacity='0.9'/>" +
        "<circle cx='200' cy='156' r='48' stroke='#c0a6e8' stroke-opacity='0.9'/>" +
        "</g>" +
        "<g stroke-width='1.4' stroke-opacity='0.4' fill='none'>" +
        "<line x1='200' y1='119' x2='152' y2='100' stroke='#7fd4e4'/>" +
        "<line x1='200' y1='119' x2='248' y2='100' stroke='#9db8ee'/>" +
        "<line x1='200' y1='119' x2='200' y2='156' stroke='#c0a6e8'/>" +
        "</g>" +
        "<circle cx='200' cy='119' r='8' fill='#eafbff'/>" +
        "<circle cx='200' cy='119' r='16' fill='none' stroke='#eafbff' stroke-opacity='0.4' stroke-width='1.5'/>" +
        "<g fill='#eafbff'>" +
        "<circle cx='152' cy='100' r='3.6'/>" +
        "<circle cx='248' cy='100' r='3.6'/>" +
        "<circle cx='200' cy='156' r='3.6'/>" +
        "</g>" +
        "<text x='88' y='104' font-size='13' text-anchor='middle' fill='#9fe4ef' font-family='Helvetica, Arial, sans-serif'>激光</text>" +
        "<text x='312' y='104' font-size='13' text-anchor='middle' fill='#b6caf5' font-family='Helvetica, Arial, sans-serif'>视觉</text>" +
        "<text x='200' y='226' font-size='13' text-anchor='middle' fill='#d2bdf0' font-family='Helvetica, Arial, sans-serif'>IMU</text>" +
        "<text x='200' y='258' font-size='19' font-weight='700' text-anchor='middle' fill='#eafbff' font-family='Helvetica, Arial, sans-serif'>融合掌舵者</text>" +
        "<text x='200' y='282' font-size='12' text-anchor='middle' fill='rgba(234,251,255,0.72)' font-family='Helvetica, Arial, sans-serif'>阶段 E · 9 篇 · 多传感器融合</text>" +
        "</svg>",
      card_file: "reference/成就卡-第五季.html"
    },
    {
      id: "ach-stage-f",
      title: "隐式织网者",
      icon: "🫧",
      paper: "阶段 F · 前沿：神经隐式与 3D 高斯",
      date: "2026-09-20",
      rarity: "legendary",
      thumb_b64:
        "data:image/svg+xml;utf8," +
        "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 400 300'>" +
        "<defs>" +
        "<linearGradient id='bgf' x1='0' y1='0' x2='1' y2='1'>" +
        "<stop offset='0%' stop-color='#1d1740'/>" +
        "<stop offset='58%' stop-color='#3f2f78'/>" +
        "<stop offset='100%' stop-color='#141029'/>" +
        "</linearGradient>" +
        "<radialGradient id='glowf' cx='50%' cy='38%' r='60%'>" +
        "<stop offset='0%' stop-color='rgba(185,163,255,0.34)'/>" +
        "<stop offset='100%' stop-color='rgba(185,163,255,0)'/>" +
        "</radialGradient>" +
        "</defs>" +
        "<rect width='400' height='300' fill='url(#bgf)'/>" +
        "<rect width='400' height='300' fill='url(#glowf)'/>" +
        "<g stroke-width='2.4'>" +
        "<ellipse cx='176' cy='104' rx='68' ry='38' transform='rotate(-24 176 104)' fill='#b9a3ff' fill-opacity='0.28' stroke='#cbb8ff' stroke-opacity='0.85'/>" +
        "<ellipse cx='228' cy='122' rx='60' ry='32' transform='rotate(32 228 122)' fill='#8fd8e8' fill-opacity='0.26' stroke='#a9e6f2' stroke-opacity='0.85'/>" +
        "<ellipse cx='196' cy='158' rx='54' ry='30' transform='rotate(8 196 158)' fill='#f0a6d8' fill-opacity='0.24' stroke='#f7bfe6' stroke-opacity='0.8'/>" +
        "<ellipse cx='250' cy='78' rx='38' ry='22' transform='rotate(-58 250 78)' fill='#cbb8ff' fill-opacity='0.20' stroke='#dcd0ff' stroke-opacity='0.7'/>" +
        "</g>" +
        "<g fill='#f4efff'>" +
        "<circle cx='176' cy='104' r='6.4'/><circle cx='228' cy='122' r='6'/><circle cx='196' cy='158' r='5.6'/><circle cx='250' cy='78' r='4.8'/>" +
        "</g>" +
        "<g fill='#e8e0ff' fill-opacity='0.5'>" +
        "<circle cx='124' cy='150' r='3'/><circle cx='286' cy='164' r='3'/><circle cx='298' cy='124' r='2.6'/><circle cx='144' cy='82' r='2.6'/><circle cx='212' cy='196' r='2.8'/><circle cx='116' cy='116' r='2.4'/>" +
        "</g>" +
        "<text x='200' y='258' font-size='19' font-weight='700' text-anchor='middle' fill='#f2eeff' font-family='Helvetica, Arial, sans-serif'>隐式织网者</text>" +
        "<text x='200' y='282' font-size='12' text-anchor='middle' fill='rgba(242,238,255,0.72)' font-family='Helvetica, Arial, sans-serif'>阶段 F · 36 篇 · 隐式与 3D 高斯</text>" +
        "</svg>",
      card_file: "reference/成就卡-第六季.html"
    }
,
    {
      id: "ach-stage-g",
      title: "关联破解者",
      icon: "🧩",
      paper: "阶段 G · 学习式方法",
      date: "2026-09-21",
      rarity: "legendary",
      thumb_b64:
        "data:image/svg+xml;utf8," +
        "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 400 300'>" +
        "<defs>" +
        "<linearGradient id='bgg' x1='0' y1='0' x2='1' y2='1'>" +
        "<stop offset='0%' stop-color='#1b2340'/>" +
        "<stop offset='58%' stop-color='#33376e'/>" +
        "<stop offset='100%' stop-color='#141a2e'/>" +
        "</linearGradient>" +
        "<radialGradient id='glowg' cx='50%' cy='36%' r='60%'>" +
        "<stop offset='0%' stop-color='rgba(160,190,255,0.32)'/>" +
        "<stop offset='100%' stop-color='rgba(160,190,255,0)'/>" +
        "</radialGradient>" +
        "</defs>" +
        "<rect width='400' height='300' fill='url(%23bgg)'/>" +
        "<rect width='400' height='300' fill='url(%23glowg)'/>" +
        "<rect x='58' y='66' width='118' height='92' rx='7' fill='rgba(240,240,255,0.10)' stroke='rgba(226,224,255,0.78)' stroke-width='2.2'/>" +
        "<rect x='224' y='66' width='118' height='92' rx='7' fill='rgba(240,240,255,0.10)' stroke='rgba(226,224,255,0.78)' stroke-width='2.2'/>" +
        "<line x1='74' y1='140' x2='160' y2='140' stroke='rgba(226,224,255,0.28)' stroke-width='1.6'/>" +
        "<line x1='240' y1='140' x2='326' y2='140' stroke='rgba(226,224,255,0.28)' stroke-width='1.6'/>" +
        "<rect x='78' y='98' width='38' height='26' rx='3' fill='rgba(240,240,255,0.18)'/>" +
        "<rect x='128' y='92' width='30' height='32' rx='3' fill='rgba(240,240,255,0.13)'/>" +
        "<rect x='248' y='98' width='32' height='26' rx='3' fill='rgba(240,240,255,0.18)'/>" +
        "<rect x='292' y='92' width='38' height='32' rx='3' fill='rgba(240,240,255,0.13)'/>" +
        "<line x1='97' y1='111' x2='264' y2='111' stroke='#8ee8b8' stroke-width='3'/>" +
        "<line x1='143' y1='108' x2='311' y2='108' stroke='#8ee8b8' stroke-width='3'/>" +
        "<line x1='97' y1='130' x2='264' y2='130' stroke='#f5cf8a' stroke-width='2.6'/>" +
        "<line x1='170' y1='86' x2='330' y2='86' stroke='rgba(255,160,150,0.85)' stroke-width='2.4' stroke-dasharray='6,5'/>" +
        "<g stroke='rgba(255,160,150,0.95)' stroke-width='2.6'>" +
        "<line x1='344' y1='132' x2='356' y2='144'/><line x1='356' y1='132' x2='344' y2='144'/>" +
        "</g>" +
        "<g fill='#f4f2ff'>" +
        "<circle cx='97' cy='111' r='4.4'/><circle cx='264' cy='111' r='4.4'/>" +
        "<circle cx='143' cy='108' r='4.2'/><circle cx='311' cy='108' r='4.2'/>" +
        "<circle cx='97' cy='130' r='3.8'/><circle cx='264' cy='130' r='3.8'/>" +
        "</g>" +
        "<g stroke='rgba(226,224,255,0.70)' stroke-width='1.8' fill='rgba(240,240,255,0.09)'>" +
        "<rect x='120' y='188' width='36' height='26' rx='3'/>" +
        "<rect x='162' y='188' width='36' height='26' rx='3'/>" +
        "<rect x='204' y='188' width='36' height='26' rx='3'/>" +
        "<rect x='246' y='188' width='36' height='26' rx='3'/>" +
        "</g>" +
        "<g stroke='rgba(226,224,255,0.38)' stroke-width='1.4'>" +
        "<line x1='127' y1='198' x2='149' y2='198'/><line x1='169' y1='198' x2='191' y2='198'/>" +
        "<line x1='211' y1='198' x2='233' y2='198'/><line x1='253' y1='198' x2='275' y2='198'/>" +
        "</g>" +
        "<text x='200' y='252' font-size='27' font-weight='700' text-anchor='middle' fill='#f2f0ff' font-family='Helvetica, Arial, sans-serif'>关联破解者</text>" +
        "<text x='200' y='280' font-size='13' text-anchor='middle' fill='rgba(238,236,255,0.72)' font-family='Helvetica, Arial, sans-serif'>阶段 G · 31 篇 · 匹配 / 检索 / 端到端 / 基础模型</text>" +
        "</svg>",
      card_file: "reference/成就卡-第七季.html"
    },
    {
      id: "ach-stage-h",
      title: "支线开拓者",
      icon: "🧭",
      paper: "阶段 H · 专题支线",
      date: "2026-09-21",
      rarity: "legendary",
      thumb_b64:
        "data:image/svg+xml;utf8," +
        "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 400 300'><defs><linearGradient id='bgh' x1='0' y1='0' x2='1' y2='1'><stop offset='0%' stop-color='#0f2f2a'/><stop offset='58%' stop-color='#1c5a4e'/><stop offset='100%' stop-color='#0a211d'/></linearGradient><radialGradient id='glowh' cx='50%' cy='36%' r='62%'><stop offset='0%' stop-color='rgba(124,224,192,0.34)'/><stop offset='100%' stop-color='rgba(124,224,192,0)'/></radialGradient></defs><rect width='400' height='300' fill='url(%23bgh)'/><rect width='400' height='300' fill='url(%23glowh)'/><circle cx='96' cy='76' r='9' fill='rgba(240,255,249,0.24)' stroke='rgba(226,255,246,0.85)' stroke-width='2'/><rect x='89' y='88' width='14' height='28' rx='6' fill='rgba(240,255,249,0.20)' stroke='rgba(226,255,246,0.72)' stroke-width='1.6'/><path d='M 70 130 L 124 130' stroke='rgba(255,190,170,0.9)' stroke-width='2.2' stroke-dasharray='5,4'/><text x='97' y='152' font-size='13' text-anchor='middle' fill='rgba(236,255,249,0.82)' font-family='Helvetica, Arial, sans-serif'>动态</text><rect x='256' y='84' width='46' height='8' rx='3' fill='rgba(240,255,249,0.26)' stroke='rgba(226,255,246,0.72)' stroke-width='1.4'/><line x1='262' y1='92' x2='262' y2='122' stroke='rgba(226,255,246,0.66)' stroke-width='1.8'/><line x1='296' y1='92' x2='296' y2='122' stroke='rgba(226,255,246,0.66)' stroke-width='1.8'/><line x1='262' y1='122' x2='296' y2='122' stroke='rgba(226,255,246,0.66)' stroke-width='1.8'/><rect x='240' y='66' width='78' height='68' rx='10' fill='none' stroke='rgba(124,224,192,0.88)' stroke-width='2' stroke-dasharray='6,4'/><text x='279' y='152' font-size='13' text-anchor='middle' fill='rgba(236,255,249,0.82)' font-family='Helvetica, Arial, sans-serif'>语义</text><rect x='62' y='190' width='34' height='24' rx='6' fill='rgba(240,255,249,0.20)' stroke='rgba(226,255,246,0.80)' stroke-width='1.8'/><circle cx='72' cy='218' r='5' fill='rgba(226,255,246,0.70)'/><circle cx='88' cy='218' r='5' fill='rgba(226,255,246,0.70)'/><rect x='132' y='190' width='34' height='24' rx='6' fill='rgba(240,255,249,0.20)' stroke='rgba(226,255,246,0.80)' stroke-width='1.8'/><circle cx='142' cy='218' r='5' fill='rgba(226,255,246,0.70)'/><circle cx='158' cy='218' r='5' fill='rgba(226,255,246,0.70)'/><line x1='98' y1='202' x2='130' y2='202' stroke='rgba(124,224,192,0.92)' stroke-width='2.4' stroke-dasharray='4,4'/><g stroke='rgba(255,190,170,0.95)' stroke-width='2.2'><line x1='108' y1='236' x2='118' y2='246'/><line x1='118' y1='236' x2='108' y2='246'/></g><text x='114' y='266' font-size='13' text-anchor='middle' fill='rgba(236,255,249,0.82)' font-family='Helvetica, Arial, sans-serif'>多机</text><rect x='240' y='188' width='76' height='46' rx='8' fill='rgba(240,255,249,0.14)' stroke='rgba(226,255,246,0.72)' stroke-width='1.8'/><circle cx='240' cy='211' r='8' fill='rgba(226,255,246,0.55)'/><g fill='rgba(160,255,214,0.95)'><circle cx='266' cy='200' r='3.2'/><circle cx='290' cy='216' r='3.2'/><circle cx='304' cy='198' r='3.2'/><circle cx='276' cy='226' r='3.2'/></g><g fill='rgba(255,170,160,0.95)'><circle cx='282' cy='206' r='3.2'/><circle cx='300' cy='228' r='3.2'/></g><text x='278' y='266' font-size='13' text-anchor='middle' fill='rgba(236,255,249,0.82)' font-family='Helvetica, Arial, sans-serif'>事件</text><text x='200' y='40' font-size='14' text-anchor='middle' fill='rgba(236,255,249,0.66)' font-family='Helvetica, Arial, sans-serif'>三条假设，四个支线</text><text x='200' y='296' font-size='13' text-anchor='middle' fill='rgba(236,255,249,0.72)' font-family='Helvetica, Arial, sans-serif'>阶段 H · 42 篇 · 动态 / 语义 / 多机 / 事件</text></svg>",
      card_file: "reference/成就卡-第八季.html"
    },
    {
      id: "ach-stage-i",
      title: "尺度校准者",
      icon: "📏",
      paper: "阶段 I · 数据集与评测",
      date: "2026-09-21",
      rarity: "legendary",
      thumb_b64:
        "data:image/svg+xml;utf8," +
        "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 400 300'><defs><linearGradient id='bgi' x1='0' y1='0' x2='1' y2='1'><stop offset='0%' stop-color='#141f36'/><stop offset='58%' stop-color='#2f4a72'/><stop offset='100%' stop-color='#0e1730'/></linearGradient><radialGradient id='glowi' cx='50%' cy='34%' r='62%'><stop offset='0%' stop-color='rgba(143,196,240,0.34)'/><stop offset='100%' stop-color='rgba(143,196,240,0)'/></radialGradient></defs><rect width='400' height='300' fill='url(%23bgi)'/><rect width='400' height='300' fill='url(%23glowi)'/><rect x='40' y='78' width='320' height='56' rx='8' fill='rgba(240,246,255,0.12)' stroke='rgba(226,238,255,0.82)' stroke-width='2.4'/><line x1='40' y1='110' x2='360' y2='110' stroke='rgba(226,238,255,0.42)' stroke-width='1.6'/><g stroke='rgba(226,238,255,0.88)' stroke-width='2.6'><line x1='90' y1='78' x2='90' y2='102'/><line x1='200' y1='78' x2='200' y2='102'/><line x1='310' y1='78' x2='310' y2='102'/></g><g stroke='rgba(226,238,255,0.45)' stroke-width='1.4'><line x1='60' y1='78' x2='60' y2='90'/><line x1='120' y1='78' x2='120' y2='90'/><line x1='150' y1='78' x2='150' y2='90'/><line x1='170' y1='78' x2='170' y2='90'/><line x1='230' y1='78' x2='230' y2='90'/><line x1='260' y1='78' x2='260' y2='90'/><line x1='280' y1='78' x2='280' y2='90'/><line x1='340' y1='78' x2='340' y2='90'/></g><text x='90' y='150' font-size='15' text-anchor='middle' fill='#9fd8ff' font-family='Helvetica, Arial, sans-serif'>毫米级</text><text x='200' y='150' font-size='15' text-anchor='middle' fill='#9fd8ff' font-family='Helvetica, Arial, sans-serif'>厘米级</text><text x='310' y='150' font-size='15' text-anchor='middle' fill='#9fd8ff' font-family='Helvetica, Arial, sans-serif'>亚米级</text><g stroke='rgba(226,238,255,0.80)' stroke-width='1.8' fill='rgba(240,246,255,0.12)'><rect x='52' y='176' width='26' height='18' rx='3'/><rect x='92' y='176' width='26' height='18' rx='3'/></g><circle cx='65' cy='185' r='4' fill='rgba(226,238,255,0.88)'/><circle cx='105' cy='185' r='4' fill='rgba(226,238,255,0.88)'/><text x='85' y='212' font-size='11.5' text-anchor='middle' fill='rgba(226,238,255,0.78)' font-family='Helvetica, Arial, sans-serif'>动捕房</text><g stroke='rgba(226,238,255,0.80)' stroke-width='1.8' fill='none'><rect x='182' y='168' width='24' height='18' rx='3' fill='rgba(240,246,255,0.14)'/><line x1='194' y1='186' x2='182' y2='208'/><line x1='194' y1='186' x2='194' y2='208'/><line x1='194' y1='186' x2='206' y2='208'/></g><circle cx='194' cy='164' r='4' fill='rgba(226,238,255,0.55)'/><text x='194' y='226' font-size='11.5' text-anchor='middle' fill='rgba(226,238,255,0.78)' font-family='Helvetica, Arial, sans-serif'>全站仪</text><g stroke='rgba(226,238,255,0.80)' stroke-width='1.8'><line x1='310' y1='170' x2='310' y2='186'/><circle cx='310' cy='166' r='7' fill='rgba(240,246,255,0.22)'/><rect x='288' y='186' width='44' height='18' rx='3' fill='rgba(240,246,255,0.14)'/><circle cx='298' cy='208' r='5' fill='rgba(226,238,255,0.60)'/><circle cx='322' cy='208' r='5' fill='rgba(226,238,255,0.60)'/></g><text x='310' y='228' font-size='11.5' text-anchor='middle' fill='rgba(226,238,255,0.78)' font-family='Helvetica, Arial, sans-serif'>RTK 天线</text><text x='200' y='262' font-size='24' font-weight='700' text-anchor='middle' fill='#f2f8ff' font-family='Helvetica, Arial, sans-serif'>尺度校准者</text><text x='200' y='286' font-size='13' text-anchor='middle' fill='rgba(238,245,255,0.72)' font-family='Helvetica, Arial, sans-serif'>阶段 I · 5 篇 · 够用比准更重要</text></svg>",
      card_file: "reference/成就卡-第九季.html"
    },
    {
      id: "ach-stage-j",
      title: "名词解码者",
      icon: "🧰",
      paper: "阶段 J · 基础零件",
      date: "2026-09-21",
      rarity: "legendary",
      thumb_b64:
        "data:image/svg+xml;utf8," +
        "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 400 300'><defs><linearGradient id='bgj' x1='0' y1='0' x2='1' y2='1'><stop offset='0%' stop-color='#3a2410'/><stop offset='58%' stop-color='#8a5a24'/><stop offset='100%' stop-color='#25160a'/></linearGradient><radialGradient id='glowj' cx='50%' cy='34%' r='62%'><stop offset='0%' stop-color='rgba(240,192,122,0.34)'/><stop offset='100%' stop-color='rgba(240,192,122,0)'/></radialGradient></defs><rect width='400' height='300' fill='url(%23bgj)'/><rect width='400' height='300' fill='url(%23glowj)'/><rect x='26' y='54' width='348' height='150' rx='14' fill='rgba(255,255,255,0.07)' stroke='rgba(255,238,214,0.55)' stroke-width='2.4'/><line x1='26' y1='92' x2='374' y2='92' stroke='rgba(255,238,214,0.26)' stroke-width='1.6'/><rect x='42' y='104' width='72' height='86' rx='8' fill='rgba(255,255,255,0.05)' stroke='rgba(240,192,122,0.44)' stroke-width='1.6'/><rect x='64' y='118' width='28' height='8' rx='2.5' fill='rgba(240,192,122,0.88)'/><rect x='64' y='132' width='28' height='8' rx='2.5' fill='rgba(240,192,122,0.66)'/><rect x='64' y='146' width='28' height='8' rx='2.5' fill='rgba(240,192,122,0.48)'/><rect x='64' y='160' width='28' height='8' rx='2.5' fill='rgba(240,192,122,0.34)'/><rect x='126' y='104' width='72' height='86' rx='8' fill='rgba(255,255,255,0.05)' stroke='rgba(240,192,122,0.44)' stroke-width='1.6'/><rect x='142' y='136' width='40' height='20' rx='4' fill='rgba(180,214,255,0.24)' stroke='rgba(180,214,255,0.72)' stroke-width='1.4'/><circle cx='154' cy='160' r='4' fill='rgba(180,214,255,0.72)'/><circle cx='170' cy='160' r='4' fill='rgba(180,214,255,0.72)'/><rect x='138' y='130' width='48' height='34' rx='3' fill='none' stroke='rgba(255,209,102,0.92)' stroke-width='2' stroke-dasharray='5 3'/><rect x='210' y='104' width='72' height='86' rx='8' fill='rgba(255,255,255,0.05)' stroke='rgba(240,192,122,0.44)' stroke-width='1.6'/><path d='M226 148 Q238 124 254 140 Q268 154 258 168 Q244 182 230 172 Q218 162 226 148 Z' fill='rgba(122,227,183,0.30)' stroke='rgba(122,227,183,0.86)' stroke-width='2'/><path d='M234 152 Q244 136 254 148 Q262 158 254 166 Q244 174 236 164 Z' fill='rgba(240,192,122,0.34)' stroke='rgba(240,192,122,0.90)' stroke-width='1.6'/><rect x='294' y='104' width='72' height='86' rx='8' fill='rgba(255,255,255,0.05)' stroke='rgba(240,192,122,0.44)' stroke-width='1.6'/><ellipse cx='330' cy='138' rx='22' ry='7.5' fill='rgba(180,214,255,0.20)' stroke='rgba(180,214,255,0.72)' stroke-width='1.6'/><rect x='308' y='138' width='44' height='24' fill='rgba(180,214,255,0.16)' stroke='rgba(180,214,255,0.60)' stroke-width='1.6'/><line x1='330' y1='162' x2='348' y2='182' stroke='rgba(255,209,102,0.90)' stroke-width='2'/><circle cx='350' cy='185' r='3.6' fill='rgba(255,209,102,0.92)'/><text x='78' y='212' font-size='12' text-anchor='middle' fill='rgba(255,246,232,0.78)' font-family='Helvetica, Arial, sans-serif'>骨架</text><text x='162' y='212' font-size='12' text-anchor='middle' fill='rgba(255,246,232,0.78)' font-family='Helvetica, Arial, sans-serif'>检测头</text><text x='246' y='212' font-size='12' text-anchor='middle' fill='rgba(255,246,232,0.78)' font-family='Helvetica, Arial, sans-serif'>分割头</text><text x='330' y='212' font-size='12' text-anchor='middle' fill='rgba(255,246,232,0.78)' font-family='Helvetica, Arial, sans-serif'>两端外围</text><text x='200' y='256' font-size='24' font-weight='700' text-anchor='middle' fill='#fff8ec' font-family='Helvetica, Arial, sans-serif'>名词解码者</text><text x='200' y='282' font-size='13' text-anchor='middle' fill='rgba(255,246,232,0.74)' font-family='Helvetica, Arial, sans-serif'>阶段 J · 11 篇 · 四族零件</text></svg>",
      card_file: "reference/成就卡-第十季.html"
    }
  ]
};
