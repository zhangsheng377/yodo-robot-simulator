// v2.8 / build 20261018-31 竖屏（600×900，(x,y)→(y,x)）测试（车 170×100；严格整车四角；R0=407.100；指挥部 140；护栏 ±73 只报警）
//   圈号：我方 4/5/6、对岸 1/2/3（1=(515,560) 2=(500,780) 3=(263,752)；对称 4↔3、5↔1、6↔2）· R0=408.100（标定参考位 (299.684,98.674)/−115.02° 反推；车长 170）
//   判定（build-28）：整个车身（170×100）与圈有任何重叠即可 · 车不出界（build-30）：整车四角严格在场内 · 镜面反弹+折线瞄准线 ·
//   散布=整条弹道平移(28px) · 按住只偏概率(±8.4px) · 落地后连续衰减弹跳约 72px（6 段）
// 运行：node test_v28.js（默认测同目录 yodo_training_v2.8_b5.html，可用 SIM_FILE 指向别处）
const path = require('path');
const fs = require('fs');
function findHarness() {
  const cands = [path.join(__dirname, 'harness2.js'), path.join(__dirname, '..', 'harness2.js')];
  for (const p of cands) if (fs.existsSync(p)) return p;
  return './harness2';
}
const { loadSim } = require(findHarness());
function defaultSimFile() {
  const fixed = path.join(__dirname, 'yodo_training_v2.8.html');
  if (fs.existsSync(fixed)) return fixed;
  const list = fs.readdirSync(__dirname).filter(f => /^yodo_training_v2\.8.*\.html$/.test(f)).sort();
  if (list.length) return path.join(__dirname, list[list.length - 1]);
  return path.join(__dirname, '..', 'yodo_training_v2.8.html');
}
const FILE = process.env.SIM_FILE ? path.resolve(process.env.SIM_FILE) : defaultSimFile();
const { api, h } = loadSim(FILE);
const S = api.state;
const html = fs.readFileSync(FILE, 'utf8');
const VER = h.env.window.__SIM_VERSION__;
const W = api.ARENA_W, H = api.ARENA_H;

let pass = 0, fail = 0;
function ok(name, cond, detail) {
  if (cond) { pass++; console.log('PASS  ' + name + (detail ? '  [' + detail + ']' : '')); }
  else { fail++; console.log('FAIL  ' + name + (detail ? '  [' + detail + ']' : '')); }
}
function frames(n, step = 16) { let t = h.clock.now; for (let i = 0; i < n; i++) { t += step; h.frame(t); } return t; }
function press(key) { h.dispatchWin('keydown', { key, preventDefault() {}, repeat: false }); }
function release(key) { h.dispatchWin('keyup', { key, preventDefault() {} }); }
const cRef = id => api.CIRCLES.find(c => c.id === id);
const OUR = api.ourCircleIds(), FAR = api.farCircleIds().slice().sort((a, b) => a - b), DEF_START = api.defaultStartId();
const CAL = api.CALIB_STATION;
const calTheta = CAL.thetaDeg * Math.PI / 180;
const C1 = cRef(1), C2 = cRef(2), C3 = cRef(3), C4 = cRef(4), C5 = cRef(5), C6 = cRef(6);
const D2R = Math.PI / 180;

h.frame(0);
api.setRandomSeed(20261018);
ok('撞护栏节流初值为负（-9999ms）→ 第一次碰到立即提示，不会被节流吞掉',
  S.lastBarrierAt < -2000, 'lastBarrierAt=' + S.lastBarrierAt);

function armAt(x, y, theta, scatter) {
  api.applyConfig(DEF_START, true); api.resetRound(true); api.startRound();
  S.car = { x, y, theta, vl: 0, vr: 0 };
  const b = { x, y, vx: 0, vy: 0, alive: true, flying: false, id: 0, handPlaced: false, originCircleId: null };
  S.balls = [b]; S.loadedBall = b; S.collectingBall = null; S.collectProgress = 2;
  S.scatter = scatter === undefined ? false : scatter;
  return b;
}
function fly(b, maxFrames) { for (let i = 0; i < (maxFrames || 900) && b.flying; i++) api.updateBalls(1 / 60); }
function pathLen(pts) { let L = 0; for (let i = 1; i < pts.length; i++) L += Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y); return L; }
function minDistToPoint(pts, cx, cy) {
  let best = 1e9;
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1], b = pts[i];
    const vx = b.x - a.x, vy = b.y - a.y, L2 = vx * vx + vy * vy || 1;
    const t = Math.max(0, Math.min(1, ((cx - a.x) * vx + (cy - a.y) * vy) / L2));
    best = Math.min(best, Math.hypot(a.x + vx * t - cx, a.y + vy * t - cy));
  }
  return best;
}
// 独立 OBB-圆相交（只用于验证"只擦角不算"这条判定）
function obbOverlapsCircle(cx, cy, theta, circle) {
  const dx = circle.x - cx, dy = circle.y - cy;
  const hx = dx * Math.cos(theta) + dy * Math.sin(theta);
  const hy = -dx * Math.sin(theta) + dy * Math.cos(theta);
  const gx = Math.max(0, Math.abs(hx) - api.CAR_LEN / 2);
  const gy = Math.max(0, Math.abs(hy) - api.CAR_WID / 2);
  return Math.hypot(gx, gy) <= circle.r;
}

// ==================== 0. 版本 / 尺寸 / 常量 ====================
ok('版本标记 v2.8+20261018-31', VER === 'v2.8+20261018-31', VER);
ok('画布 600×900 竖屏', W === 600 && H === 900 && html.includes('width="600" height="900"'), W + '×' + H);
ok('尺寸：车 170×100（build-31 缩短）/ 球 r16 / 命中 33 / 目标物 26±23 / 桥 200×150',
  api.CAR_LEN === 170 && api.CAR_WID === 100 && api.BALL_R === 16 &&
  api.TARGET_HIT_RADIUS === 33 && api.TARGET_SIZE === 26 &&
  api.BRIDGE.len === 200 && api.BRIDGE.wid === 150);
ok('常量：SPREAD_R=28 / HOLD_BIAS_MAX=16.8 / 弹跳 40×0.45 停 0.5 / 反弹上限 5',
  api.SPREAD_R === 28 && Math.abs(api.HOLD_BIAS_MAX - 16.8) < 1e-9 &&
  api.BOUNCE0 === 40 && Math.abs(api.BOUNCE_DECAY - 0.45) < 1e-9 &&
  Math.abs(api.BOUNCE_MIN - 0.5) < 1e-9 && api.BOUNCE_MAX === 20 && api.WALL_BOUNCE_MAX === 5);
ok('两个后轮：WHEEL_AXLE_X=-25（build-31 缩短后按后悬 60 折算）/ WHEEL_SPAN=86，无前轮常量',
  api.WHEEL_AXLE_X === -25 && api.WHEEL_SPAN === 86 && !/FRONT_WHEEL/.test(html));
ok('车长派生：TAIL_OFFSET=101（85+16）/ CAR_HALF_DIAG=√(85²+50²)≈98.62',
  api.TAIL_OFFSET === 101 && Math.abs(api.CAR_HALF_DIAG - Math.hypot(85, 50)) < 1e-9);

// ==================== 1. 编号 / 对称 / 坐标变换自洽 ====================
// 变换前（900×600 横屏）坐标表与变换后应有位置
const OLD = { 1: [752, 263], 2: [340, 85], 3: [560, 515], 4: [148, 337], 5: [120, 100], 6: [780, 500] };
const T = p => [p[1], p[0]];
// 定稿编号 → 该位置来自旧坐标表的哪个号（变换后按现场重读印刷号，号随位置重排）
const NEW_FROM_OLD = { 1: 3, 2: 6, 3: 1, 4: 4, 5: 2, 6: 5 };
{
  const pos = {};
  api.CIRCLES.forEach(c => { pos[c.id] = [c.x, c.y]; });
  const eq = (a, b) => a[0] === b[0] && a[1] === b[1];
  ok('定稿编号：我方 4/5/6、对岸 1/2/3',
    JSON.stringify(OUR) === '[4,5,6]' && JSON.stringify(FAR) === '[1,2,3]' && DEF_START === 4,
    'OUR=' + OUR + ' FAR=' + FAR);
  ok('坐标（build-11）：(4)337,148 (5)85,340 (6)100,120 / (1)515,560 (2)500,780 (3)263,752',
    eq(pos[4], [337, 148]) && eq(pos[5], [85, 340]) && eq(pos[6], [100, 120]) &&
    eq(pos[1], [515, 560]) && eq(pos[2], [500, 780]) && eq(pos[3], [263, 752]));
  ok('坐标集合 = 旧坐标 (x,y)→(y,x) 变换（一个不差）', (() => {
    const a = api.CIRCLES.map(c => c.x + ',' + c.y).sort().join('|');
    const b = Object.values(OLD).map(T).map(p => p[0] + ',' + p[1]).sort().join('|');
    return a === b;
  })());
  ok('逐个位置变换自洽（新号 ← 旧号按定稿映射）',
    Object.keys(NEW_FROM_OLD).every(id => eq(pos[id], T(OLD[NEW_FROM_OLD[id]]))));
  ok('对称（build-11 重算）：4↔3、5↔1、6↔2（关于场地中心 (300,450) 180°）',
    C4.x + C3.x === 600 && C4.y + C3.y === 900 &&
    C5.x + C1.x === 600 && C5.y + C1.y === 900 &&
    C6.x + C2.x === 600 && C6.y + C2.y === 900);
  ok('半径 58 全部不变', api.CIRCLES.every(c => c.r === 58));
}
ok('基地：我方 (600,0) 右上、对岸 (0,900) 左下',
  api.BASE_US.x === 600 && api.BASE_US.y === 0 && api.BASE_THEM.x === 0 && api.BASE_THEM.y === 900);
ok('桥：心 (363,446.5)、角度 48°、dir=(0.669,0.743)',
  api.BRIDGE.cx === 363 && api.BRIDGE.cy === 446.5 && api.BRIDGE.angleDeg === 48 &&
  Math.abs(api.BRIDGE.dir[0] - 0.669) < 1e-9 && Math.abs(api.BRIDGE.dir[1] - 0.743) < 1e-9);
ok('河：上下岸线 y(x)；对岸 = y > botY(x)',
  api.inRiver(300, 500) && !api.inRiver(300, 60) && api.onFarSide(300, 700) && !api.onFarSide(300, 100));
ok('12 个目标物只在对岸 1/2/3 各 4 个',
  S.targets.length === 12 && FAR.every(i => api.targetsOfCircle(i).length === 4) &&
  OUR.every(i => api.targetsOfCircle(i).length === 0));
ok('6 个球初始位置：都在我方半场、不在河里、离圈心 > 74',
  api.BALL_INIT.every(b => !api.onFarSide(b.x, b.y) && !api.inRiver(b.x, b.y) &&
    api.CIRCLES.every(c => Math.hypot(c.x - b.x, c.y - b.y) > c.r + 15)));

// ==================== 1b. 收球判定（v2.8 build-10：只收车头收球器前方一小段）====================
// 车体坐标：车头 +x，车头端 L=85（build-31）；收球区 x∈[85−24, 85+20]、|y|≤50
function collectProbe(lx, ly, thetaDeg) {
  api.applyConfig(DEF_START, true); api.resetRound(true); api.startRound();
  const th = (thetaDeg || 0) * D2R;
  S.car = { x: 300, y: 450, theta: th, vl: 0, vr: 0 };
  const b = {
    x: 300 + Math.cos(th) * lx - Math.sin(th) * ly,
    y: 450 + Math.sin(th) * lx + Math.cos(th) * ly,
    vx: 0, vy: 0, alive: true, flying: false, id: 0, handPlaced: false, originCircleId: null,
  };
  S.balls = [b]; S.loadedBall = null; S.collectingBall = null; S.collectProgress = 0;
  api.tryCollect();
  return S.collectingBall === b;
}
ok('收球区常量：车头端再外伸 20px（16~30 内）、内侧 24px、半宽 50（=车身宽 100）',
  api.COLLECT_FWD === 20 && api.COLLECT_BACK === 24 && api.COLLECT_HALF_W === 50);
ok('车头正前 121mm（球心距车心 105）= 外伸边界，能收', collectProbe(105, 0));
ok('车头正前再远 1px（106）不收——范围不做远', !collectProbe(106, 0));
ok('车头正前 61px（贴收球口内侧边界）能收', collectProbe(61, 0));
ok('车心正前方 50px（在车身里、未到收球口）不收', !collectProbe(50, 0));
ok('车尾方向 60px 的球不收（旧 74px 整圆会收，回归项）', !collectProbe(-60, 0));
ok('车身正侧面 30px 的球不收', !collectProbe(0, 30));
ok('车头前 85px、侧向 50px（收球口边缘）能收', collectProbe(85, 50));
ok('车头前 85px、侧向 51px（超出车身宽）不收', !collectProbe(85, 51));
ok('判定随车头方向旋转：车头 -83° 时正前方 100px 能收、正后方 100px 不收',
  collectProbe(100, 0, -83.02) && !collectProbe(-100, 0, -83.02));

// ==================== 2. R0=407.100 与标定（build-31：车长 170 / 后轴 -25 后重解）====================
const R0 = api.SHOT_RANGE_BASE;
ok('R0 = 408.101（球出生点口径，校准值）= computeBaseRange() 由标定参考位反推',
  Math.abs(R0 - 408.101) < 0.02, R0.toFixed(4));
ok('从车尾外沿发球口量 = 424.101', Math.abs(api.calibRangeFromMuzzle() - 424.101) < 0.02,
  api.calibRangeFromMuzzle().toFixed(4));
ok('4 号圈心→1 号圈心 = 448.807（标定基准）',
  Math.abs(Math.hypot(C1.x - C4.x, C1.y - C4.y) - 448.807) < 0.05,
  Math.hypot(C1.x - C4.x, C1.y - C4.y).toFixed(3));
{
  // 标定参考位（build-31：车体压圈位；旧"后轮压圈边"位在 L=170 下最多只能给 404.89，保不住 28/30）
  ok('标定参考位（build-31 甲方案：车心贴 4 号圈外缘、车身压住圈心）：(299.684,98.674)、车头 -115.02°、目标圈 1',
    Math.abs(CAL.x - 299.684) < 0.002 && Math.abs(CAL.y - 98.674) < 0.002 &&
    Math.abs(CAL.thetaDeg + 115.02) < 0.002);
  const corners = [[api.CAR_LEN / 2, api.CAR_WID / 2], [api.CAR_LEN / 2, -api.CAR_WID / 2],
    [-api.CAR_LEN / 2, api.CAR_WID / 2], [-api.CAR_LEN / 2, -api.CAR_WID / 2]].map(([lx, ly]) => ({
    x: CAL.x + lx * Math.cos(calTheta) - ly * Math.sin(calTheta),
    y: CAL.y + lx * Math.sin(calTheta) + ly * Math.cos(calTheta),
  }));
  const margin = Math.min(...corners.map(p => Math.min(p.x, 600 - p.x, p.y, 900 - p.y)));
  ok('标定参考位整车四角在场内（最小余量 ' + margin.toFixed(3) + 'px = 边界夹取的 0.5px）',
    margin >= 0.4999 && margin < 0.51 && api.carObbInside(CAL.x, CAL.y, calTheta, api.CAR_BOUND_MARGIN - 1e-9),
    'margin=' + margin.toFixed(3));
  S.car = { x: CAL.x, y: CAL.y, theta: calTheta, vl: 0, vr: 0 };
  ok('标定参考位可发射且 4 号圈心落在车体内（车身压圈 58px、obbDist=0）',
    (api.carInAnyCircle() || {}).id === 4 &&
    api.obbPointDist(CAL.x, CAL.y, calTheta, C4.x, C4.y) === 0,
    'obbDist=' + api.obbPointDist(CAL.x, CAL.y, calTheta, C4.x, C4.y).toFixed(2));
  const cO = Math.hypot(CAL.x - C4.x, CAL.y - C4.y);
  ok('标定参考位：车心贴在 4 号圈外缘（距圈心 ' + cO.toFixed(2) + 'px，圈外 3.85px）',
    Math.abs(cO - 61.85) < 0.02 && cO > 58, cO.toFixed(2) + 'px');
  const P = api.predictPoint(0, calTheta, CAL.x, CAL.y);
  const d = Math.hypot(P.x - C1.x, P.y - C1.y);
  ok('校准一致性（R0 由参考位反推，落点重合是同义反复、不算实测）：落点距 1 号圈心 ' + d.toFixed(3) + 'px（≤0.6 容差）',
    CAL.targetId === 1 && d <= 0.6 && P.type === 'circle' && P.circleId === 1, d.toFixed(3) + 'px');
  // 参考位余量正好 0.5px（= 边界夹取余量），游戏内边界夹取不会推动它
  const cc = { x: CAL.x, y: CAL.y, theta: calTheta };
  api.clampCarByBounds(cc);
  ok('标定参考位余量 = 边界夹取余量 0.5px，游戏内夹取不会推动它',
    Math.hypot(cc.x - CAL.x, cc.y - CAL.y) < 1e-6);
}

// ==================== 3. 发射判定：整个车身（170×100）与圈有任何重叠即可（build-28 用户改口径）====================
{
  api.applyConfig(4, true); api.resetRound(true); api.startRound();
  // ① 车心在圈内（车体必然压圈）→ 可发
  S.car = { x: C4.x, y: C4.y, theta: 0, vl: 0, vr: 0 };
  ok('① 车心在圈内 → 可发射', (api.carInAnyCircle() || {}).id === 4);
  // ② 车心出圈、两后轮也在圈外，但车头端压圈 → 也能发（本轮改口径的核心；旧口径会拒发）
  S.car = { x: C4.x + 140, y: C4.y, theta: Math.PI, vl: 0, vr: 0 };
  const centerOut2 = Math.hypot(S.car.x - C4.x, S.car.y - C4.y) > 58;
  const wheelsOut2 = [1, -1].every(sg => {
    const lx = api.WHEEL_AXLE_X, ly = sg * api.WHEEL_SPAN / 2;
    const wx = S.car.x + lx * Math.cos(Math.PI) - ly * Math.sin(Math.PI);
    const wy = S.car.y + lx * Math.sin(Math.PI) + ly * Math.cos(Math.PI);
    return Math.hypot(wx - C4.x, wy - C4.y) > 58;
  });
  const od2 = api.obbPointDist(S.car.x, S.car.y, Math.PI, C4.x, C4.y);
  ok('② 车心出圈 140px、两后轮都在圈外，但车头端压圈 ' + (58 - od2).toFixed(1) + 'px → 可发射（旧口径会拒发）',
    centerOut2 && wheelsOut2 && (api.carInAnyCircle() || {}).id === 4,
    '车心出圈=' + centerOut2 + ' 后轮出圈=' + wheelsOut2 + ' obbDist=' + od2.toFixed(2));
  // ③ 整车完全在圈外（车头端离圈边 5px）→ 拒发
  S.car = { x: C4.x + 95 + 58 + 5, y: C4.y, theta: Math.PI, vl: 0, vr: 0 };
  ok('③ 整车完全在圈外（车头端离圈边 5px，obbDist=63）→ 拒发',
    api.carInAnyCircle() === null,
    'obbDist=' + api.obbPointDist(S.car.x, S.car.y, Math.PI, C4.x, C4.y).toFixed(2));
  // ④ 边界按"任何重叠"判：压进 0.5px 能发；差 0.5px 没压上不能发
  S.car = { x: C4.x + api.CAR_LEN / 2 + 58 - 0.5, y: C4.y, theta: Math.PI, vl: 0, vr: 0 };
  const inHalf = (api.carInAnyCircle() || {}).id === 4;
  S.car = { x: C4.x + api.CAR_LEN / 2 + 58 + 0.5, y: C4.y, theta: Math.PI, vl: 0, vr: 0 };
  const outHalf = api.carInAnyCircle() === null;
  ok('④ 边界按"任何重叠"判：压进 0.5px 能发、差 0.5px 不能发', inHalf && outHalf,
    'in=' + inHalf + ' out=' + outHalf);
  // ⑤ 完全在圈外（远离所有圈）
  S.car = { x: 500, y: 300, theta: 0, vl: 0, vr: 0 };
  ok('⑤ 远离所有圈 → 拒发', api.carInAnyCircle() === null);
  ok('页面文案已改成"车体压圈"口径（不再写"车心/后轮在圈内"才能发）',
    (html.includes('车体压到圈里') || html.includes('车身与圈有任何重叠')) && !html.includes('或至少一个后轮在圈里才能发射'));
}

// ==================== 4. 撞边框镜面反弹 ====================
{
  const tr1 = api.traceShot(300, 5, 0, 1, 950);
  ok('反弹 1 次：路径总长严格 = R', Math.abs(pathLen(tr1.points) - 950) < 1e-6 && tr1.bounces === 1,
    'len=' + pathLen(tr1.points).toFixed(3) + ' bounces=' + tr1.bounces);
  ok('反弹点贴在下边框 y=900', Math.abs(tr1.points[1].y - 900) < 1e-9 && Math.abs(tr1.points[1].x - 300) < 1e-9);
  ok('落点镜像正确：y = 900 − (950 − 895) = 845', Math.abs(tr1.landing.y - 845) < 1e-6, tr1.landing.y.toFixed(2));
  const tr2 = api.traceShot(300, 5, 0, 1, 1850);
  ok('反弹 2 次：路径总长仍 = R，落点 y=55',
    tr2.bounces === 2 && Math.abs(pathLen(tr2.points) - 1850) < 1e-6 && Math.abs(tr2.landing.y - 55) < 1e-6);
  const tr3 = api.traceShot(300, 5, 0, 1, 2750);
  ok('反弹 3 次：路径总长仍 = R', tr3.bounces === 3 && Math.abs(pathLen(tr3.points) - 2750) < 1e-6);
  let reflectOk = true, nB = 0;
  for (let k = 0; k < 24; k++) {
    const th = k * Math.PI / 12;
    const tr = api.traceShot(300, 450, Math.cos(th), Math.sin(th), 2000);
    for (let i = 1; i < tr.points.length - 1; i++) {
      const p = tr.points[i], a = tr.points[i - 1], b = tr.points[i + 1];
      const vi = { x: p.x - a.x, y: p.y - a.y }, vo = { x: b.x - p.x, y: b.y - p.y };
      const li = Math.hypot(vi.x, vi.y) || 1, lo = Math.hypot(vo.x, vo.y) || 1;
      vi.x /= li; vi.y /= li; vo.x /= lo; vo.y /= lo;
      const onV = Math.abs(p.x) < 1e-6 || Math.abs(p.x - W) < 1e-6;
      if (onV) { if (!(Math.abs(vi.x + vo.x) < 1e-9 && Math.abs(vi.y - vo.y) < 1e-9)) reflectOk = false; }
      else { if (!(Math.abs(vi.y + vo.y) < 1e-9 && Math.abs(vi.x - vo.x) < 1e-9)) reflectOk = false; }
      nB++;
    }
  }
  ok('镜面反射：折点法向分量反向、切向不变（24 方向 × ' + nB + ' 折点）', reflectOk && nB > 20);
  const trMax = api.traceShot(300, 450, 0.02, 0.9999, 4000);
  ok('反弹次数上限 5', trMax.bounces <= api.WALL_BOUNCE_MAX, 'bounces=' + trMax.bounces);
}

// ==================== 5. 16 方向弹道总长恒 = R0 ====================
{
  let lenOk = true, minB = 99, maxB = -1;
  for (let k = 0; k < 16; k++) {
    const P = api.predictPoint(0, k * Math.PI / 8, C6.x, C6.y);
    if (Math.abs(pathLen(P.points) - R0) > 1e-6) lenOk = false;
    minB = Math.min(minB, P.bounces); maxB = Math.max(maxB, P.bounces);
  }
  ok('16 方向弹道折线总长恒 = R0', lenOk, R0.toFixed(2) + 'px');
  ok('有反弹也有不反弹的方向', minB === 0 && maxB >= 1, 'bounces ' + minB + '–' + maxB);
}

// ==================== 6. 瞄准线折线 + 零长度段处理 ====================
{
  // 车在 4 号圈心、车头朝 -x（theta=π）→ 车尾朝 +x，撞右边框反弹
  armAt(C4.x, C4.y, Math.PI);
  const P = api.predictPoint(0);
  S.lastPredict = P;
  const AL = api.aimLine();
  ok('瞄准线：1 次反弹 → 3 点、折点贴右边框 x=600',
    AL.points.length === 3 && AL.bounces === 1 && Math.abs(AL.points[1].x - 600) < 1e-9,
    'points=' + AL.points.length + ' bounce=' + (AL.points[1] && AL.points[1].x.toFixed(1)));
  ok('瞄准线起点=发球口、终点=预判落点',
    Math.hypot(AL.from.x - P.spawn.x, AL.from.y - P.spawn.y) < 1e-9 &&
    Math.hypot(AL.to.x - P.x, AL.to.y - P.y) < 1e-9);
  ok('各段方向 = 弹道各段方向', (() => {
    for (let i = 1; i < AL.points.length; i++) {
      const a = P.points[i - 1], b2 = P.points[i];
      const dx = b2.x - a.x, dy = b2.y - a.y, L = Math.hypot(dx, dy) || 1;
      const ax = AL.points[i].x - AL.points[i - 1].x, ay = AL.points[i].y - AL.points[i - 1].y, A2 = Math.hypot(ax, ay) || 1;
      if (Math.abs(dx / L - ax / A2) > 1e-9 || Math.abs(dy / L - ay / A2) > 1e-9) return false;
    }
    return true;
  })());
  ok('不用斜线直连（起终点直线距离 < 弹道长）',
    Math.hypot(P.x - P.spawn.x, P.y - P.spawn.y) < R0 - 1);
  // 零长度段：出生点贴边框朝外（traceShot 会产生重复点）
  const tr = api.traceShot(300, 0, 0, -1, 150);
  const fake = {
    points: tr.points, spawn: { x: 300, y: 0 }, x: tr.landing.x, y: tr.landing.y,
    type: 'ground', bounces: tr.bounces, endDir: tr.dir,
  };
  S.lastPredict = fake;
  const AL2 = api.aimLine();
  let minSeg = 1e9;
  for (let i = 1; i < AL2.points.length; i++) minSeg = Math.min(minSeg, Math.hypot(AL2.points[i].x - AL2.points[i - 1].x, AL2.points[i].y - AL2.points[i - 1].y));
  ok('零长度段已过滤（贴边/擦角时不画假撞墙方块）',
    minSeg > 0.5 && AL2.points.length === 2 && AL2.bounces === 0,
    'points=' + AL2.points.length + ' minSeg=' + minSeg.toFixed(2));
}

// ==================== 7. 实际发射：沿折线飞、落点=预判 ====================
{
  const b = armAt(C4.x, C4.y, Math.PI, false);
  const P = api.predictPoint(0);
  api.tryFire(0);
  ok('实际飞行轨迹 = 预判折线（含反弹点）',
    b.flight && b.flight.points.length === P.points.length &&
    b.flight.points.every((p, i) => Math.hypot(p.x - P.points[i].x, p.y - P.points[i].y) < 1e-9));
  ok('飞行路径总长 = R0', Math.abs(b.flight.total - R0) < 1e-9 && Math.abs(pathLen(b.flight.points) - R0) < 1e-6);
  let insideOk = true, nearWall = false;
  for (let i = 0; i < 400 && b.flying; i++) {
    api.updateBalls(1 / 60);
    if (b.x < -1e-6 || b.x > W + 1e-6 || b.y < -1e-6 || b.y > H + 1e-6) insideOk = false;
    if (b.x > W - 12) nearWall = true;
  }
  ok('反弹飞行全程不出场、真的飞到边框再折返', insideOk && nearWall);
  ok('关随机时第一落点 = 预判落点', b.landing && Math.hypot(b.landing.x - P.x, b.landing.y - P.y) < 1e-6);
}

// ==================== 8. 落地后连续衰减弹跳（build-29：40/18/8.1/3.65/1.64/0.74，总约 72.1px）====================
{
  const total = api.bounceTotal();
  ok('弹跳总前进距离 ≈72.12px（6 段：40 / 18 / 8.1 / 3.65 / 1.64 / 0.74）',
    Math.abs(total - 72.123) < 0.05, total.toFixed(3));
  const tr = api.bounceTrajectory(100, 120, { x: 0.5183017160933442, y: 0.855197831554018 });
  ok('弹跳序列逐步衰减（每段 = 上一段 ×0.45，单段 <0.5px 停）',
    tr.length === 6 && tr.every((p, i) => Math.abs(p.step - 40 * Math.pow(0.45, i)) < 1e-9) &&
    tr.every(p => p.step >= api.BOUNCE_MIN),
    tr.map(p => p.step.toFixed(2)).join(' / '));
  ok('弹跳段数不超过上限（不会无限弹）', tr.length <= api.BOUNCE_MAX);
  ok('弹跳序列各段之和 = bounceTotal（轨迹和总长一致）',
    Math.abs(tr.reduce((acc, p) => acc + p.step, 0) - total) < 1e-6);
  // 撞墙反射：贴着右边框朝墙弹，落点必须被折回场内（不再"贴边夹住"）
  const refl = api.bounceTrajectory(560, 450, { x: 1, y: 0 });
  // 反射不损失长度：把每段落点连起来的路径长度应 = 各段步长之和（clamp 实现会变短）
  let pathLen = 0, prevX = 560, prevY = 450;
  for (const p of refl) { pathLen += Math.hypot(p.x - prevX, p.y - prevY); prevX = p.x; prevY = p.y; }
  ok('弹跳段撞边框按镜面反射：落点始终在场内、不穿出去、且路径长度守恒',
    refl.length === 6 && refl.every(p => p.x >= 0 && p.x <= 600 && p.y >= 0 && p.y <= 900) &&
    Math.abs(pathLen - refl.reduce((acc, p) => acc + p.step, 0)) < 1e-6,
    'pathLen=' + pathLen.toFixed(3) + ' / steps=' + refl.reduce((acc, p) => acc + p.step, 0).toFixed(3) +
    ' refl=' + refl.map(p => '(' + p.x.toFixed(0) + ',' + p.y.toFixed(0) + ')').join(' '));
  // 实弹：第一落点后连续弹跳，球停在弹完的位置
  const b = armAt(C4.x, C4.y, Math.PI, false);
  const P = api.predictPoint(0);
  api.tryFire(0); fly(b);
  const moved = Math.hypot(b.x - P.x, b.y - P.y);
  ok('实弹：第一落点后继续弹跳（最终位移量级 = 弹跳总长，且不飞出场）',
    moved > 60 && moved <= total + 1 && b.flying === false,
    '位移 ' + moved.toFixed(2) + 'px / 弹跳总长 ' + total.toFixed(2));
  ok('第一落点 = 预判圆心（预判圈仍等于第一落点，这条不变）',
    Math.hypot(b.landing.x - P.x, b.landing.y - P.y) < 1e-6);
  ok('预判带弹跳落点数据（淡色小点）：6 个、全在场内',
    P.bouncePoints && P.bouncePoints.length === 6 &&
    P.bouncePoints.every(q => q.x >= 0 && q.x <= 600 && q.y >= 0 && q.y <= 900));
  // 弹跳撞墙反射：直接构造"首落点贴右边框、方向朝墙"的弹跳轨迹
  {
    const qs = api.bounceTrajectory(598, 450, { x: 1, y: 0 });
    const q0 = qs[0];
    ok('弹跳第一点撞右边框后镜面折回场内（x 折回变小、不穿出）',
      q0.x < 598 && q0.x >= 0 && q0.x <= 600 && Math.abs(q0.y - 450) < 1e-9,
      'firstBounce=(' + q0.x.toFixed(2) + ',' + q0.y.toFixed(2) + ')');
    const qL = api.bounceTrajectory(2, 450, { x: -1, y: 0 });
    ok('弹跳第一点撞左边框后镜面折回场内（x 折回变大、不穿出）',
      qL[0].x > 2 && qL[0].x >= 0 && qL[0].x <= 600,
      'firstBounce=(' + qL[0].x.toFixed(2) + ',' + qL[0].y.toFixed(2) + ')');
    let sum = 0;
    for (const q of qs) sum += q.step;
    ok('弹跳撞墙后路径长度守恒 = 各段 step 之和（镜面反射不损失距离）', Math.abs(sum - api.bounceTotal()) < 1e-6,
      'sum=' + sum.toFixed(3) + ' total=' + api.bounceTotal().toFixed(3));
  }
  ok('预判小点用青色、并在页面注明"预估弹跳落点/实际落点有散布"',
    html.includes("'rgba(0,229,255,0.9)'") && html.includes('预估') && html.includes('实际落点有散布'));
  ok('页面写明 6→2 是修正既有口径错误、2→6 无目标物不计分',
    html.includes('修正既有口径错误') && html.includes('本就不计分'));
  // ===== build-29：两张表分开 + 只有 6→2 因弹跳翻盘 + 第二个目标不可倒 =====
  {
    const rows2 = api.reachMatrix();
    const FAR_ROWS = rows2.filter(r => FAR.includes(r.to));
    const flipped = FAR_ROWS.filter(r => r.knock.firstLanding === 0 && r.knock.bounceMax > 0)
      .map(r => r.from + '>' + r.to);
    ok('全量扫描：只有 6→2 因弹跳翻成"可击倒"，没有别的对翻盘',
      flipped.length === 1 && flipped[0] === '6>2', flipped.join(' '));
    // 6→2：弹跳落点到 4 个目标里只有 1 个 <33，第二个最近也 >33（永远够不到）
    const r62 = rows2.find(r => r.from === 6 && r.to === 2);
    const C2 = cRef(2);
    const tg = api.TARGET_OFFSETS.map(([dx, dy]) => ({ x: C2.x + dx, y: C2.y + dy }));
    const ds = tg.map(t => {
      let best = Infinity;
      for (const q of (api.predictPoint(0, r62.station.theta, r62.station.x, r62.station.y).bouncePoints || [])) {
        const dd = Math.hypot(q.x - t.x, q.y - t.y);
        if (dd < best) best = dd;
      }
      return best;
    }).sort((a, b) => a - b);
    ok('6→2 弹跳只能击倒 1 个：最近目标 ' + ds[0].toFixed(1) + 'px <33、第二近 ' + ds[1].toFixed(1) + 'px >33',
      ds[0] < 33 && ds[1] > 33);
  }
  // ===== 预判弹跳点：某段落河就截断，不画"进河后还在弹"的点 =====
  {
    const P = api.predictPoint(0, 15 * D2R, cRef(2).x, cRef(2).y);
    ok('预判弹跳点遇河截断（落河那点及之后都不画）',
      P.type !== 'river' && P.bouncePoints.length >= 1 && P.bouncePoints.length < 6 &&
      P.bouncePoints.every(q => !(api.inRiver(q.x, q.y) && !api.onBridge(q.x, q.y))),
      'dots=' + P.bouncePoints.length);
  }
  // ===== 记账三条：弹跳倒 2 个 → score +40、shotsHit 只 +1、散点 hit=true、landing.type=circle =====
  {
    // 动态找"首落 ground、靠弹跳落进对岸圈并同时压到 ≥2 个目标"的站位（球在 4/5/6 发）
    let found = null;
    for (const car of [4, 5, 6]) {
      const A = cRef(car);
      for (let gi = 0; gi < 720 && !found; gi++) {
        const th = gi * Math.PI / 360;
        const PP = api.predictPoint(0, th, A.x, A.y);
        if (PP.type !== 'ground') continue;
        for (const q of (PP.bouncePoints || [])) {
          for (const C of api.CIRCLES) {
            if (!api.isFarCircle(C)) continue;
            let n = 0;
            for (const [dx, dy] of api.TARGET_OFFSETS) {
              if (Math.hypot(q.x - (C.x + dx), q.y - (C.y + dy)) < api.TARGET_HIT_RADIUS) n++;
            }
            if (n >= 2) { found = { car, A, th, C, n }; break; }
          }
          if (found) break;
        }
      }
      if (found) break;
    }
    ok('构造：首落点是 ground、靠弹跳才击倒 2 个（球在 4/5/6 号圈发）', !!found,
      found ? '圈' + found.car + ' th=' + (found.th * 180 / Math.PI).toFixed(1) + ' 目标圈' + found.C.id : '未找到');
    if (found) {
      api.applyConfig(4, true); api.resetRound(true); api.startRound();
      S.car = { x: found.A.x, y: found.A.y, theta: found.th, vl: 0, vr: 0 };
      const bb = { x: found.A.x, y: found.A.y, vx: 0, vy: 0, alive: true, flying: false, id: 0, handPlaced: false, originCircleId: null };
      S.balls = [bb]; S.loadedBall = bb; S.collectingBall = null; S.collectProgress = 2; S.scatter = false;
      api.tryFire(0); fly(bb);
      const downN = api.targetsOfCircle(found.C.id).filter(t => t.down).length;
      const dot = S.spreadDots[S.spreadDots.length - 1];
      ok('记账三条：弹跳倒 2 个 → score 40、shotsHit 只 +1、散点 hit=true、landing.type=circle',
        downN === 2 && S.score === 40 && S.shotsHit === 1 &&
        bb.landing.type === 'circle' && dot && dot.hit === true,
        'down=' + downN + ' score=' + S.score + ' shotsHit=' + S.shotsHit + ' type=' + bb.landing.type);
    }
  }
  // 每段都判定：用 6→2 的"弹跳进圈"站位实弹，必须击倒 2 号圈近侧目标、且一发只算一次命中
  const r62 = api.reachMatrix().find(r => r.from === 6 && r.to === 2);
  const b2 = armAt(r62.station.x, r62.station.y, r62.station.theta, false);
  api.tryFire(0); fly(b2);
  const down2 = api.targetsOfCircle(2).filter(t => t.down).length;
  ok('每段弹跳落点都判定：6→2 第一落点打不到，但弹跳进圈击倒近侧 1 个、+20 分（写死）',
    down2 === 1 && S.score === 20, '击倒 ' + down2 + ' 个 / 本发得分 ' + S.score);
  ok('多段弹跳只算一次命中（shotsHit 只 +1）', S.shotsHit === 1, 'shotsHit=' + S.shotsHit);
  // build-29 记账统一：弹跳后才击倒的那发，landing.type / 散点 / shotsHit 三者一致
  {
    const row = api.reachMatrix().find(r => r.from === 6 && r.to === 2);
    const b3 = armAt(row.station.x, row.station.y, row.station.theta, false);
    api.tryFire(0); fly(b3);
    const dot = S.spreadDots[S.spreadDots.length - 1];
    ok('记账统一：弹跳后才击倒的一发，landing.type=circle、散点 hit=true、shotsHit=1',
      b3.landing.type === 'circle' && dot && dot.hit === true && S.shotsHit === 1,
      'type=' + b3.landing.type + ' dotHit=' + (dot && dot.hit) + ' shotsHit=' + S.shotsHit);
  }
  // 每段判定 / 弹跳落河丢球：搜索一发"首落在岸上、但某段弹跳落进河里"的球
  {
    let found = null;
    for (const A of api.CIRCLES) {
      for (let gi = 0; gi < 16 && !found; gi++) {
        const th = gi * 22.5 * D2R;
        const P = api.predictPoint(0, th, A.x, A.y);
        if (P.type === 'river') continue;
        const boom = api.bounceTrajectory(P.x, P.y, P.endDir || { x: 0, y: 0 });
        const riverStep = boom.findIndex(q => api.inRiver(q.x, q.y) && !api.onBridge(q.x, q.y));
        if (boom.length && riverStep >= 0) found = { A, th, P, riverStep };
      }
      if (found) break;
    }
    ok('能构造"首落在岸上、弹跳阶段落进河里"的球', !!found,
      found ? '圈' + found.A.id + ' 第 ' + (found.riverStep + 1) + ' 段入河' : '未找到');
    if (found) {
      api.applyConfig(4, true); api.resetRound(true); api.startRound();
      S.car = { x: found.A.x, y: found.A.y, theta: found.th, vl: 0, vr: 0 };
      const bb = { x: S.car.x, y: S.car.y, vx: 0, vy: 0, alive: true, flying: false, id: 0, handPlaced: false, originCircleId: null };
      S.balls = [bb]; S.loadedBall = bb; S.collectingBall = null; S.collectProgress = 2; S.scatter = false;
      api.tryFire(0); fly(bb);
      ok('弹跳落河丢球：球 alive=false，且停在入河那一段', bb.alive === false && bb.bounceLandings && bb.bounceLandings.length >= 1);
    }
  }
}

// ==================== 9. 落点判定按最终落点：飞行穿圈不计分（搜索）====================
{
  let found = null;
  for (const A of api.CIRCLES) {
    for (let gi = 0; gi < 8 && !found; gi++) {
      const ang = gi * 0.9, rr = 25 * (gi % 3);
      const px = A.x + Math.cos(ang) * rr, py = A.y + Math.sin(ang) * rr;
      for (let k = 0; k < 240 && !found; k++) {
        const th = k * 1.5 * D2R;
        const P = api.predictPoint(0, th, px, py);
        if (P.type === 'circle') continue;
        for (const B of api.CIRCLES) {
          if (!api.isFarCircle(B)) continue;
          if (minDistToPoint(P.points, B.x, B.y) < 45 && Math.hypot(P.x - B.x, P.y - B.y) > 58) {
            found = { x: px, y: py, th, P, B }; break;
          }
        }
      }
    }
    if (found) break;
  }
  ok('能构造"弹道穿过对岸圈、落点在圈外"的弹道', !!found,
    found ? '穿圈' + found.B.id : '未找到');
  if (found) {
    const b = armAt(found.x, found.y, found.th, false);
    let minDist = 1e9;
    api.tryFire(0);
    for (let i = 0; i < 500 && b.flying; i++) {
      api.updateBalls(1 / 60);
      for (const id of FAR) { const c = cRef(id); minDist = Math.min(minDist, Math.hypot(b.x - c.x, b.y - c.y)); }
    }
    ok('球真的从对岸圈里飞过（最近 < 45px）', minDist < 45, minDist.toFixed(1) + 'px');
    ok('穿圈不计分（落点圈外 → down=0 / score=0）',
      api.targetsDownCount() === 0 && S.score === 0);
  }
}

// ==================== 10. ground / river 分类与掉河 ====================
{
  let riverCase = null;
  for (const A of api.CIRCLES) {
    for (let gi = 0; gi < 8 && !riverCase; gi++) {
      const ang = gi * 0.9, rr = 20 * (gi % 3);
      const px = A.x + Math.cos(ang) * rr, py = A.y + Math.sin(ang) * rr;
      for (let k = 0; k < 360; k++) {
        const P = api.predictPoint(0, k * D2R, px, py);
        if (P.type === 'river') { riverCase = { x: px, y: py, th: k * D2R, P }; break; }
      }
    }
    if (riverCase) break;
  }
  ok('能找到落点在河里（river）的弹道', !!riverCase,
    riverCase ? '落点(' + riverCase.P.x.toFixed(0) + ',' + riverCase.P.y.toFixed(0) + ')' : '');
  if (riverCase) {
    const b = armAt(riverCase.x, riverCase.y, riverCase.th, false);
    api.tryFire(0); fly(b);
    ok('落河 → 判 river、球丢（alive=false）', b.landing.type === 'river' && b.alive === false);
  }
}

// ==================== 11. 击倒：4→1（R0=407.100 下仍能压 1 号圈心）→ 4 全倒 ====================
{
  const r = api.reachMatrix().find(x => x.from === 4 && x.to === 1);
  const b = armAt(r.station.x, r.station.y, r.station.theta, false);
  const P = api.predictPoint(0);
  api.tryFire(0); fly(b);
  ok('4→1：压圈心、4 全倒 +80',
    P.type === 'circle' && P.circleId === 1 && api.targetsOfCircle(1).filter(t => t.down).length === 4 && S.score === 80,
    '落点距 1 号心 ' + Math.hypot(P.x - C1.x, P.y - C1.y).toFixed(2) + 'px down=' +
    api.targetsOfCircle(1).filter(t => t.down).length + ' score=' + S.score);
}

// ==================== 12. 散布 = 整条弹道平移 ====================
ok('起点圈只允许我方圈', (() => { api.applyConfig(1, true); return S.config.startId === 4; })(),
  'startId=' + S.config.startId);
{
  const r = api.reachMatrix().find(x => x.from === 4 && x.to === 1);
  api.applyConfig(4, true); api.resetRound(true); api.startRound();
  S.car = { x: r.station.x, y: r.station.y, theta: r.station.theta, vl: 0, vr: 0 };
  S.scatter = true;
  const P = api.predictPoint(0);
  let maxR = 0, lenOk = true, inField = true;
  const n = 1200;
  for (let i = 0; i < n; i++) {
    const b = { x: 0, y: 0, vx: 0, vy: 0, alive: true, flying: false, id: 0, handPlaced: false, originCircleId: null };
    S.balls = [b]; S.loadedBall = b;
    api.tryFire(0);
    const rr = Math.hypot(b.flight.toX - P.x, b.flight.toY - P.y);
    if (rr > maxR) maxR = rr;
    if (Math.abs(pathLen(b.flight.points) - R0) > 1e-6) lenOk = false;
    for (const p of b.flight.points) if (p.x < -1e-9 || p.x > W + 1e-9 || p.y < -1e-9 || p.y > H + 1e-9) inField = false;
  }
  ok('散布：1200 发落点全在预判圆心 28px 内', maxR <= api.SPREAD_R + 1e-6, '最大 ' + maxR.toFixed(2) + 'px');
  ok('散布：每发弹道总长仍严格 = R0', lenOk);
  ok('散布：每发弹道都在场内', inField);
}

// ==================== 13. 按住只偏圈内概率 ====================
function sampleBias(hold, n) {
  const r = api.reachMatrix().find(x => x.from === 4 && x.to === 1);
  api.applyConfig(4, true); api.resetRound(true); api.startRound();
  S.car = { x: r.station.x, y: r.station.y, theta: r.station.theta, vl: 0, vr: 0 };
  S.scatter = true;
  const P = api.predictPoint(hold);
  const ux = P.endDir.x, uy = P.endDir.y;
  let sum = 0, maxR = 0, out = 0;
  for (let i = 0; i < n; i++) {
    const b = { x: 0, y: 0, vx: 0, vy: 0, alive: true, flying: false, id: 0, handPlaced: false, originCircleId: null };
    S.balls = [b]; S.loadedBall = b;
    api.tryFire(hold);
    const dx = b.flight.toX - P.x, dy = b.flight.toY - P.y;
    sum += dx * ux + dy * uy;
    const rr = Math.hypot(dx, dy);
    if (rr > maxR) maxR = rr;
    if (rr > api.SPREAD_R + 1e-6) out++;
  }
  return { mean: sum / n, maxR, out };
}
{
  const b0 = sampleBias(0, 1000), b1 = sampleBias(api.HOLD_T_MAX / 2, 1000), b2 = sampleBias(api.HOLD_T_MAX, 1000);
  ok('按住 0 秒：均值沿前方偏 -8.4px', Math.abs(b0.mean + 8.4) < 0.9, b0.mean.toFixed(2) + 'px');
  ok('按住一半：偏置 ≈ 0', Math.abs(b1.mean) < 0.9, b1.mean.toFixed(2) + 'px');
  ok('按满：均值偏 +8.4px', Math.abs(b2.mean - 8.4) < 0.9, b2.mean.toFixed(2) + 'px');
  ok('偏置随按住时长单调后移', b0.mean < b1.mean - 4 && b1.mean < b2.mean - 4,
    b0.mean.toFixed(2) + ' → ' + b1.mean.toFixed(2) + ' → ' + b2.mean.toFixed(2));
  ok('3000 发全部在 28px 圆内', b0.out === 0 && b1.out === 0 && b2.out === 0,
    '最大 ' + Math.max(b0.maxR, b1.maxR, b2.maxR).toFixed(2) + 'px');
  ok('按住期间圈心/射程不变', (() => {
    const ref = api.predictPoint(0, 0, C6.x, C6.y);
    return [0, 0.2, 0.6, 1.5, 5].every(hd => {
      const P = api.predictPoint(hd, 0, C6.x, C6.y);
      return Math.hypot(P.x - ref.x, P.y - ref.y) < 1e-12;
    });
  })());
  ok('页面没有"力度/概率"UI（唯一的 range 是速度条，不得有蓄力/力度条）',
    !html.includes('蓄力') && !html.includes('力度条') && !html.includes('百分比') &&
    (html.match(/type="range"/g) || []).length === 1 && html.includes('id="spd-range"'));
}

// ==================== 14. 可达矩阵（R0=407.100，车体 OBB 压圈即可发 + 整车四角在场内，30 有向对全量，build-31 重算）====================
{
  const rows = api.reachMatrix();
  ok('可达矩阵：30 个有向对全量返回（不是按距离推）', rows.length === 30, 'rows=' + rows.length);
  const reach = rows.filter(r => r.reach);
  const pairKey = rs => rs.map(r => r.from + '>' + r.to).sort().join(',');
  // ===== 独立期望清单（写死，不用 reachMatrix 自证）=====
  ok('压心 28 对 = 独立期望清单（写死）',
    pairKey(rows.filter(r => r.reach && r.minMiss < 1)) ===
    '1>2,1>3,1>4,1>5,1>6,2>1,2>3,2>4,2>5,3>1,3>2,3>4,3>5,3>6,4>1,4>2,4>3,4>5,4>6,5>1,5>2,5>3,5>4,5>6,6>1,6>3,6>4,6>5',
    pairKey(rows.filter(r => r.reach && r.minMiss < 1)));
  ok('仅进圈 0 对（新车体口径下能进圈的都能压心）', rows.filter(r => r.reach && r.minMiss >= 1).length === 0,
    pairKey(rows.filter(r => r.reach && r.minMiss >= 1)));
  ok('打不到 2 对 = 独立期望清单（写死：2↔6）',
    pairKey(rows.filter(r => !r.reach)) === '2>6,6>2', pairKey(rows.filter(r => !r.reach)));
  ok('28 个有向对可达（不可达 1 对：2↔6 双向）', reach.length === 28,
    '不可达：' + rows.filter(r => !r.reach).map(r => r.from + '>' + r.to).join(' '));
  // 每个可达对：站位确实"车体压住出发圈"、真能发射、落点压目标圈心、与 minMiss 一致
  let stOk = true; const bad = [];
  for (const r of reach) {
    const A = cRef(r.from), B = cRef(r.to);
    const overlapA = api.carObbOverlapsCircle(r.station.x, r.station.y, r.station.theta, A, -0.5);
    S.car = { x: r.station.x, y: r.station.y, theta: r.station.theta, vl: 0, vr: 0 };
    const anyJ = api.carInAnyCircle();
    const P = api.predictPoint(0, r.station.theta, r.station.x, r.station.y);
    const dd = Math.hypot(P.x - B.x, P.y - B.y);
    if (!overlapA || !anyJ || dd > 58 || Math.abs(dd - r.minMiss) > 0.6) {
      stOk = false;
      bad.push(r.from + '>' + r.to + ':overlapA=' + overlapA + ',fire=' + (anyJ && anyJ.id) + ',d=' + dd.toFixed(1));
    }
  }
  ok('28 组站位实测：车体都压住出发圈、都能发射、落点都压目标圈心且与 minMiss 一致', stOk, bad.join(' '));
  // 我方 → 对岸 9 组：8 组压心，只有 6→2 打不到
  const m = (a, b) => rows.find(r => r.from === a && r.to === b).minMiss;
  const our2far = [];
  for (const a of OUR) for (const b of FAR) our2far.push([a, b, m(a, b)]);
  ok('我方 4/5/6 → 对岸 1/2/3 共 9 组：8 组能压圈心，只有 6→2 打不到',
    our2far.filter(x => x[2] < 0.5).length === 8 &&
    our2far.filter(x => x[2] < 0.5).map(x => x[0] + '>' + x[1]).sort().join(',') ===
      '4>1,4>2,4>3,5>1,5>2,5>3,6>1,6>3',
    our2far.map(x => x[0] + '→' + x[1] + ':' + x[2].toFixed(1)).join(' '));
  ok('4→2 现在能压圈心（旧 18/6/6 里"打不到"作废）',
    m(4, 2) < 0.5 && rows.find(r => r.from === 4 && r.to === 2).reach);
  ok('6→3 现在能压圈心（旧"打不到"作废）',
    m(6, 3) < 0.5 && rows.find(r => r.from === 6 && r.to === 3).reach);
  ok('6→2 第一落点打不到（R0=407.1 后最近首落距圈心 ' + m(6, 2).toFixed(2) + 'px，>58 出圈）',
    m(6, 2) > 100 && !rows.find(r => r.from === 6 && r.to === 2).reach, m(6, 2).toFixed(2));
  ok('2↔6 双向都不可达', [[2, 6], [6, 2]].every(([a, b]) => !rows.find(r => r.from === a && r.to === b).reach));
  // build-29：矩阵保留"第一落点"分类，另给 bounce 列（弹跳后可达）
  const r62b = rows.find(r => r.from === 6 && r.to === 2);
  ok('6→2 弹跳后可达：第一落点仍打不到（最佳首落距圈心约 119px），但弹跳能进 2 号圈近侧（最深约 10.4px）',
    !r62b.reach && !!r62b.bounce && r62b.bounce.inCircle &&
    Math.abs(r62b.bounce.firstMiss - 119.34) < 1.0 && Math.abs(r62b.bounce.depth - 10.40) < 0.5,
    'firstMiss=' + (r62b.bounce && r62b.bounce.firstMiss.toFixed(2)) +
    ' depth=' + (r62b.bounce && r62b.bounce.depth.toFixed(2)));
  ok('可达行的 bounce 列：首落已压心，弹跳确实还在圈内（6 段）',
    rows.filter(r => r.reach).every(r => r.bounce && r.bounce.inCircle && r.bounce.steps === 6));
  // build-29 两套口径：①第一落点压心（28/0/2） ②含弹跳击倒（任一段落点距目标 <33；**只有对岸圈有目标物**）
  const FAR_ROWS = rows.filter(r => FAR.includes(r.to));
  const OUR_ROWS = rows.filter(r => OUR.includes(r.to));
  ok('含弹跳击倒（第二套口径）：以对岸圈为目标的 15 对全部能击倒 ≥1 个目标',
    FAR_ROWS.length === 15 && FAR_ROWS.every(r => r.knock && r.knock.any > 0),
    pairKey(FAR_ROWS.filter(r => !(r.knock && r.knock.any > 0))));
  ok('以我方圈为目标的 15 对"击倒"没有意义（我方圈没有目标物，矩阵标 noTargets）',
    OUR_ROWS.every(r => r.knock && r.knock.noTargets === true && r.knock.any === 0));
  ok('可达行（第一落点压心、且目标是对岸圈）按首落就击倒 4 个目标',
    rows.filter(r => r.reach && FAR.includes(r.to)).every(r => r.knock.firstLanding === 4));
  ok('6→2：第一落点击倒 0 个，弹跳能击倒近侧 1 个（写死，不写 1~2 个）；2→6 无目标物无意义', (() => {
    const a = rows.find(r => r.from === 6 && r.to === 2), b = rows.find(r => r.from === 2 && r.to === 6);
    return a.knock.firstLanding === 0 && a.knock.bounceMax === 1 && a.knock.any === 1 &&
      b.knock.noTargets === true && b.knock.any === 0;
  })(), JSON.stringify(rows.find(r => r.from === 6 && r.to === 2).knock));
  // 实战：4→1、5→3 压圈心 + 4 全倒（散射关）
  for (const [a, b] of [[4, 1], [5, 3]]) {
    const r = rows.find(x => x.from === a && x.to === b);
    const bb = armAt(r.station.x, r.station.y, r.station.theta, false);
    api.tryFire(0);
    fly(bb);
    const down = api.targetsOfCircle(b).filter(x => x.down).length;
    ok('实战：' + a + '→' + b + ' 按站位发射、落点压圈心并击倒 4 个',
      r.minMiss < 0.5 && down === 4, 'down=' + down + ' score=' + S.score);
  }
}

// ==================== 15. 过桥 / 车手感 / 文案 / 版本 ====================
function driveAlongBridge(offsetP, thetaOff, maxFrames, stopT) {
  const ang = api.bridgeAngle();
  const cx = api.BRIDGE.cx, cy = api.BRIDGE.cy;
  const perpX = -api.BRIDGE.dir[1], perpY = api.BRIDGE.dir[0];
  const px = cx + api.BRIDGE.dir[0] * -260 + perpX * offsetP;
  const py = cy + api.BRIDGE.dir[1] * -260 + perpY * offsetP;
  api.applyConfig(4, true); api.resetRound(true); api.startRound();
  S.lastBarrierAt = -9999;    // 关掉 1200ms 节流（harness 的 performance.now 是静态桩），让每轮首撞都能计数
  S.car = { x: px, y: py, theta: ang + thetaOff, vl: 0, vr: 0 };
  S.keys = { up: true }; S.gpKeys = {};
  let water = 0;
  for (let i = 0; i < maxFrames; i++) {
    if (api.inRiver(S.car.x, S.car.y) && !api.onBridge(S.car.x, S.car.y)) water++;
    if (stopT !== undefined && api.bridgeLocal(S.car.x, S.car.y).t > stopT) break;
    api.updateCar(1 / 60);
  }
  S.keys = {};
  return {
    crossed: api.onFarSide(S.car.x, S.car.y), water, hits: S.barrierHits,
    tEnd: api.bridgeLocal(S.car.x, S.car.y).t,
  };
}
const straight = driveAlongBridge(0, 0, 900);
ok('车（170×100）沿 48° 桥正对能过河、不落水', straight.crossed && straight.water === 0, '挡板=' + straight.hits);

// ===== 15b. 桥挡板不阻挡推进（用户现场反馈"上不了桥"的根因回归；实车驾驶，不是只测函数）=====
// 旧实现：撞挡板就回退位置 → 可通行窗口只有 ±10px / ±2°，摇杆打不进桥。
// build-31 起：撞护栏**只计数 + 报警，完全不减速、不改方向**（用户要求，别干扰操作）；仍不阻挡推进。
{
  const cases = [];
  for (const p of [0, 8, -8, 12, -12, 20, -20]) cases.push(['p=' + p, p, 0]);
  for (const d of [3, -3, 5, -5]) cases.push(['车头偏 ' + d + '°', 0, d * D2R]);
  for (const [name, p, off] of cases) {
    const r = driveAlongBridge(p, off, 1200);
    ok('挡板不阻挡：' + name + ' 能过桥（撞挡板 ' + r.hits + ' 次、落水 ' + r.water + ' 帧）',
      r.crossed && r.water === 0);
  }
  // 挡板仍要有计数（p=20 时车身必然压到挡板带）
  // 注意：计数有 1200ms 节流，而 harness 的 performance.now 是静态桩，先手动重置节流时间戳
  S.lastBarrierAt = -9999;
  const hit25 = driveAlongBridge(25, 0, 1200);
  ok('撞护栏仍计数（p=25 命中次数 > 0）', hit25.hits > 0, '命中=' + hit25.hits + ' 次');
  // build-31：撞护栏**一帧都不减速、不改方向**——同一初始状态下"压护栏"与"不压护栏"逐帧结果必须完全相同
  function oneFrameAt(p) {
    api.applyConfig(4, true); api.resetRound(true); api.startRound();
    S.lastBarrierAt = -9999;
    const angB = api.bridgeAngle();
    const bx = api.BRIDGE.cx + api.BRIDGE.dir[0] * -20 + (-api.BRIDGE.dir[1]) * p;
    const by = api.BRIDGE.cy + api.BRIDGE.dir[1] * -20 + (api.BRIDGE.dir[0]) * p;
    S.car = { x: bx, y: by, theta: angB, vl: 0, vr: 0 };
    S.keys = { up: true }; S.gpKeys = {};
    const x0 = S.car.x, y0 = S.car.y;
    api.updateCar(1 / 60);
    S.keys = {};
    return { x: S.car.x, y: S.car.y, x0, y0, th: S.car.theta, vl: S.car.vl, vr: S.car.vr, hits: S.barrierHits };
  }
  const fClean = oneFrameAt(0), fRail = oneFrameAt(-25);
  ok('撞护栏后加速度/方向一帧都不受影响（速度/朝向/单帧位移与不撞时完全相同）',
    fRail.hits > 0 && fClean.hits === 0 &&
    Math.abs(fRail.vl - fClean.vl) < 1e-12 && Math.abs(fRail.vr - fClean.vr) < 1e-12 &&
    Math.abs(fRail.th - fClean.th) < 1e-12 &&
    Math.abs((fRail.x - fRail.x0) - (fClean.x - fClean.x0)) < 1e-12 &&
    Math.abs((fRail.y - fRail.y0) - (fClean.y - fClean.y0)) < 1e-12,
    'rail vl=' + fRail.vl.toFixed(3) + ' clean vl=' + fClean.vl.toFixed(3) + ' hits=' + fRail.hits);
  ok('撞护栏不再减速：代码里已无 railSlowT / 减半分支',
    !html.includes('railSlowT') && !html.includes('每次擦到都减速') && html.includes('撞护栏只报警、不减速、不挡路'));
  // 桥入口不再误报：从岸上开到水面段入口前，p=0/±8、车头偏 0/±3° 全程 0 次计数
  for (const p of [0, 8, -8]) {
    for (const offDeg of [0, 3, -3]) {
      api.applyConfig(4, true); api.resetRound(true); api.startRound();
      S.lastBarrierAt = -9999;
      const angE = api.bridgeAngle() + offDeg * D2R;
      const sxE = api.BRIDGE.cx + api.BRIDGE.dir[0] * -190 + (-api.BRIDGE.dir[1]) * p;
      const syE = api.BRIDGE.cy + api.BRIDGE.dir[1] * -190 + (api.BRIDGE.dir[0]) * p;
      S.car = { x: sxE, y: syE, theta: angE, vl: 0, vr: 0 };
      S.keys = { up: true }; S.gpKeys = {};
      let entered = false;
      for (let i = 0; i < 1200; i++) {
        api.updateCar(1 / 60);
        if (api.bridgeLocal(S.car.x, S.car.y).t > -110) { entered = true; break; }
      }
      S.keys = {};
      ok('桥入口不再误报：p=' + p + ' 车头偏 ' + offDeg + '° 从岸上开到水面段入口前（t>-110）0 次撞护栏',
        entered && S.barrierHits === 0, 'hits=' + S.barrierHits);
    }
  }
}
// ===== 15c. build-26：桥侧挡板只覆盖水面段（用户现场纠正："桥两侧挡板没那么长，只有水面上的才有"）=====
// 旧实现：挡板沿桥长整段 180px（t=±90），桥两端岸上也画着挡板、也做碰撞。
// 现规则：挡板只在"水面段"存在（t 区间由校准后的河岸线反算），桥两端岸上敞口、无碰撞；
//        撞上只报警+计数、不减速不挡路（build-31 规则）；桥中心/48°/宽 150 没动，护栏外移到 ±73。
{
  const half = api.BRIDGE.len / 2;
  const sp1 = api.RAIL_SPAN[1], spm1 = api.RAIL_SPAN[-1];
  const L1 = sp1[1] - sp1[0], Lm1 = spm1[1] - spm1[0];
  ok('挡板只覆盖水面段：长度 ≈89.17 / 81.43px（旧实现整段 180px）',
    Math.abs(L1 - 89.17) < 0.2 && Math.abs(Lm1 - 81.43) < 0.2,
    'L+=' + L1.toFixed(2) + ' L-=' + Lm1.toFixed(2));
  ok('挡板两端都比桥长轴端内缩 50px 以上（岸上敞口）',
    (sp1[0] + half) > 50 && (half - sp1[1]) > 50 && (spm1[0] + half) > 50 && (half - spm1[1]) > 50,
    '缺口 +=(' + (sp1[0] + half).toFixed(1) + ' / ' + (half - sp1[1]).toFixed(1) + ') -=(' +
    (spm1[0] + half).toFixed(1) + ' / ' + (half - spm1[1]).toFixed(1) + ')');
  const angR = api.bridgeAngle();
  const perpX = -api.BRIDGE.dir[1], perpY = api.BRIDGE.dir[0];
  const off = api.BRIDGE_RAIL.centerOffset;
  const railPt = (t, s) => [api.BRIDGE.cx + t * api.BRIDGE.dir[0] + s * off * perpX,
    api.BRIDGE.cy + t * api.BRIDGE.dir[1] + s * off * perpY];
  const hitAt = (t, s) => { const [x, y] = railPt(t, s); return api.railLocalHit(x, y, 1) === s; };
  ok('水面段挡板线命中（t=0、±40 附近，两侧都命中）',
    hitAt(0, 1) && hitAt(0, -1) && hitAt(-40, 1) && hitAt(35, -1));
  ok('桥两端岸上段同一条挡板延长线上没有挡板（t=-80 / t=+70，两侧都不命中）',
    !hitAt(-80, 1) && !hitAt(-80, -1) && !hitAt(70, 1) && !hitAt(70, -1));
  const tEdge = sp1[0];
  ok('挡板起止边界正确（水面段外不判、内判）', !hitAt(tEdge - 2.5, 1) && hitAt(tEdge + 2.5, 1),
    't0=' + tEdge.toFixed(2));
  ok('桥几何：中心 (363,446.5)、48°、长 200/宽 150；挡板 build-31 外移到 ±71 厚 8（净宽 134，外表面齐桥沿 ±75）',
    api.BRIDGE.cx === 363 && api.BRIDGE.cy === 446.5 && api.BRIDGE.len === 200 &&
    api.BRIDGE.wid === 150 && api.BRIDGE_RAIL.centerOffset === 71 && api.BRIDGE_RAIL.thick === 8);
  ok('护栏净宽 134（中线 ±71、厚 8 → 内表面 ±67；车宽 100 左右各 17px，允许偏角≈12°）+ 外表面齐桥沿',
    2 * (api.BRIDGE_RAIL.centerOffset - api.BRIDGE_RAIL.thick / 2) === 134 &&
    api.BRIDGE_RAIL.centerOffset + api.BRIDGE_RAIL.thick / 2 === 75);
  // 实车 1：从岸上直接上桥（两侧都测，含大偏移），必须能开上桥并过去
  for (const p of [0, 30, -30, 50, -50]) {
    const r = driveAlongBridge(p, 0, 1400);
    ok('从岸上直接上桥：p=' + p + ' 能过桥（撞挡板 ' + r.hits + ' 次、落水 ' + r.water + ' 帧）',
      r.crossed && r.water === 0);
  }
  // 实车 2：从水面段侧面蹭到挡板 —— 计数仍在，但不减速，照样走完并下桥
  S.lastBarrierAt = -9999;
  const scrape = driveAlongBridge(20, 0, 1400, 60);
  ok('水面段侧面蹭挡板：仍计数（' + scrape.hits + ' 次）且能走完下桥',
    scrape.hits > 0 && scrape.crossed && scrape.water === 0, 'tEnd=' + scrape.tEnd.toFixed(1));
  // 实车 3：走完水面段后下桥 —— 终点已越过挡板水面段末端（到对岸岸上段）
  ok('走完水面段后下桥：终点 t=' + scrape.tEnd.toFixed(1) + ' 已越过水面段末端 ' + sp1[1].toFixed(1),
    scrape.tEnd > sp1[1] + 15 && api.onFarSide(S.car.x, S.car.y));
  // 挡板 t 区间断言（批评家独立反算：s=+1 [-47.5,+41.0]、s=-1 [-42.0,+39.0]，差 ≤1px）
  ok('挡板 t 区间（build-31 外移到 ±71 后自动重算）：s=+1 [-48.84,40.33]、s=-1 [-42.31,39.12]',
    Math.abs(sp1[0] + 48.84) <= 0.5 && Math.abs(sp1[1] - 40.33) <= 0.5 &&
    Math.abs(spm1[0] + 42.31) <= 0.5 && Math.abs(spm1[1] - 39.12) <= 0.5,
    'sp1=[' + sp1[0].toFixed(2) + ',' + sp1[1].toFixed(2) + '] spm1=[' + spm1[0].toFixed(2) + ',' + spm1[1].toFixed(2) + ']');
  // 实车 5：水面段以外的岸上（t=-120，桥长轴端外 20px）横向穿过挡板延长线 → 不触发计数/减速。
  // 注意位置要选在"离水面段超过半个车身投影(halfT≈50)"的地方，否则 OBB 外扩会伸进水面段（批评家提醒）。
  function driveLateralAtT(t0, pFrom, pTo, maxFrames) {
    const angP = api.bridgeAngle() + Math.PI / 2;   // 车头朝 +p 方向，横着穿过挡板延长线
    const perpX = -api.BRIDGE.dir[1], perpY = api.BRIDGE.dir[0];
    const x0 = api.BRIDGE.cx + t0 * api.BRIDGE.dir[0] + pFrom * perpX;
    const y0 = api.BRIDGE.cy + t0 * api.BRIDGE.dir[1] + pFrom * perpY;
    api.applyConfig(4, true); api.resetRound(true); api.startRound();
    S.lastBarrierAt = -9999;
    S.car = { x: x0, y: y0, theta: angP, vl: 0, vr: 0 };
    S.keys = { up: true }; S.gpKeys = {};
    let water = 0, pMax = pFrom;
    for (let i = 0; i < maxFrames; i++) {
      if (api.inRiver(S.car.x, S.car.y) && !api.onBridge(S.car.x, S.car.y)) water++;
      const qp = api.bridgeLocal(S.car.x, S.car.y).p;
      if (qp > pMax) pMax = qp;
      if (qp >= pTo) break;
      api.updateCar(1 / 60);
    }
    S.keys = {};
    return { hits: S.barrierHits, water, pMax };
  }
  const lateral = driveLateralAtT(-120, -70, 70, 1200);
  ok('水面段以外的岸上横向蹭挡板延长线：不触发（t=-120、p -70→+70，实测 pMax=' + lateral.pMax.toFixed(1) + '）',
    lateral.hits === 0 && lateral.water === 0 && lateral.pMax > 60,
    'hits=' + lateral.hits + ' water=' + lateral.water);
  // 调研员回归②的可成立版本：车长 190，横向时车身沿桥轴投影 halfT=50，所以"不触发"要求车心离水面段端 >halfT：
  // 我方侧 t ≤ -97.5、对岸侧 t ≥ +90.6；取 t=-110（12.5px 余量）横着从 p=-70 穿到 +70。
  // 调研员原文的 t=-60 / t=+60 两个位置都不成立（t=-60 车身伸到 -10、t=+60 车身伸到 +10，都已进入水面段，触发是合法的）。
  const lateral110 = driveLateralAtT(-110, -70, 70, 1200);
  ok('水面段外 t=-110 岸上横向蹭挡板：不触发（调研员回归②；t=-60/+60 因车长 190 的 OBB 外扩会伸入水面段，不成立）',
    lateral110.hits === 0 && lateral110.water === 0 && lateral110.pMax > 60,
    'hits=' + lateral110.hits + ' pMax=' + lateral110.pMax.toFixed(1));
  // 两侧挡板并集应落在调研员给的落地值 [-48,+42] 内，同时覆盖两侧实测水面段
  const railUnion = [Math.min(sp1[0], spm1[0]), Math.max(sp1[1], spm1[1])];
  ok('两侧挡板并集 ' + railUnion[0].toFixed(2) + '~' + railUnion[1].toFixed(2) +
    ' 覆盖两侧水面段（build-31 外移到 ±71 后并集 [-48.9,+40.4]）',
    railUnion[0] <= -48.6 && railUnion[0] >= -49.1 && railUnion[1] >= 40.0 && railUnion[1] <= 40.6,
    'union=[' + railUnion[0].toFixed(2) + ',' + railUnion[1].toFixed(2) + ']');
  // 实车 4：车头偏 ±5° 从岸上上桥照样过（build-25 规则保留）
  for (const d of [5, -5]) {
    const r = driveAlongBridge(0, d * D2R, 1400);
    ok('车头偏 ' + d + '° 从岸上上桥能过（撞挡板 ' + r.hits + ' 次）', r.crossed && r.water === 0);
  }
}

api.applyConfig(4, true); api.resetRound(true); api.startRound();
S.car = { x: 300, y: 200, theta: 0, vl: 0, vr: 0 };
S.keys = {};
const x0 = S.car.x;
press('ArrowUp'); frames(10, 16);
const moved = S.car.x - x0;
release('ArrowUp'); frames(1, 16);
const xStop = S.car.x; frames(3, 16);
ok('车手感：按住才动、松手立刻停', moved > 5 && Math.abs(S.car.x - xStop) < 1e-9, 'dx=' + moved.toFixed(1));
ok('面板文案：射程约 407px + 撞边框反弹 + 上下半场',
  html.includes('408px') && html.includes('撞到场地边框会反弹') && html.includes('河把场地分成<b>上下两半</b>'));
ok('页面写成新口径：车体压圈即可发 + 9 组里 8 组压心 + 6→2 第一落点打不到但弹跳能进近侧',
  html.includes('车身（170×100）与圈有任何重叠') && html.includes('8 组能压圈心') &&
  html.includes('只有 6→2 是第一落点打不到') && html.includes('408px') && html.includes('差约 120px'));
ok('页面写明连续衰减弹跳（6 段约 72px、撞墙反射、每段判定、淡色小点）',
  html.includes('连续衰减弹跳') && html.includes('约 72px') &&
  html.includes('每段落点都判定') && html.includes('淡色小点'));
ok('页面把“压圈心=4 全倒”改成散布口径（无散布 4 全倒 / 开散布平均 2~3 个）',
  html.includes('无散布时 4 全倒') && html.includes('开散布平均击倒 2~3 个'));
ok('页面 4→3 旧说法（差 28.5px / 击倒 1~2 个）已清除（build-28 起 4→3 能压圈心）',
  !html.includes('差约 28.5px') && !html.includes('击倒 1~2 个') && html.includes('4→3'));
ok('页面写清严格车不出界（整车 170×100 四角在场内、车头/车尾不许越边）与新标定说法（车心贴 4 号圈外缘、车身压住圈心）',
  html.includes('整车（170×100 四个角）都不会出场地') && html.includes('车头/车尾任何部分都不许越线') &&
  html.includes('车心贴在 4 号圈外缘、车身压住圈心') && html.includes('旧"外侧后轮压圈边"姿势最多只能到 R0≈404.5'));
ok('页面旧 ③ 口径（车心+两后轮/悬空可越边/半出圈）已清除',
  !html.includes('车心+两个后轮不会出场地') && !html.includes('悬空部分可越过边框上方') &&
  !html.includes('车后半出圈、只剩一对后轮在圈内') && !html.includes('73.2px'));
ok('页面写明“撞护栏只报警、不减速、不挡路”与净宽 138（用户可见规则）',
  html.includes('撞护栏只报警、不减速、不挡路') && html.includes('桥面净宽 134px'));
ok('页面写明“两侧挡板只有水面那一段，桥两端岸上敞开”（build-26 用户现场纠正）',
  html.includes('两侧挡板只有水面那一段') && html.includes('桥两端在岸上敞开'));
ok('撞护栏报警文案已去掉"减速中"、提示节流 2s（2 秒内只提示一次）',
  html.includes('撞到护栏') && !html.includes('减速中') && html.includes('< 2000) return') &&
  html.includes('报警 2 秒内只提示一次'));

ok('页面不再写旧的 4→3“打不到圈心/差 28.5px”（build-28 起 4→3 能压圈心）',
  !html.includes('4→3 打不到圈心但能进圈') && !html.includes('差约 28.5px'));
ok('页面同侧互打文案正确（对岸互打有分、我方无分，不再说"互相打不到"）',
  html.includes('对岸 1/2/3 互打可以得分') && !html.includes('同侧圈之间互相打不到'));
ok('2/3 圈号已定稿（页面不再出现 50/50 字样）', !html.includes('50/50'));
ok('版本标记写进页面（v2.8 · 20261018-31；标题/HUD/ver-tag/APP_BUILD 四处一致）', html.includes('v2.8 · 20261018-31') && html.includes('<title>有度积木 - 比赛训练模拟器 v2.8 · 20261018-31（竖屏）</title>') && html.includes("APP_BUILD = '20261018-31';"));
ok('页面写明收球只收车头（车尾/两侧收不到）', html.includes('收球只收车头收球器正前方一小段') && html.includes('车尾和两侧收不到'));
ok('车尾不再画深灰黑块（旧 #37474f 出球槽已删）',
  !html.includes("ctx.fillStyle = '#37474f';\n  ctx.fillRect(-L, -W * 0.45") && html.includes('#cfd8dc'));

// ==================== 15e. 车不出界（build-30：严格整车 170×100 四角在场内，车头/车尾也不许越边）====================
{
  const W = api.ARENA_W, H = api.ARENA_H;
  const MAR = api.CAR_BOUND_MARGIN;
  const inside0 = () => api.carObbInside(S.car.x, S.car.y, S.car.theta, MAR - 1e-6);
  const cornersOf = (x, y, th) => {
    const c = Math.cos(th), s = Math.sin(th);
    const HL = api.CAR_LEN / 2, HW = api.CAR_WID / 2;
    return [[HL, HW], [HL, -HW], [-HL, HW], [-HL, -HW]].map(([lx, ly]) => ({ x: x + lx * c - ly * s, y: y + lx * s + ly * c }));
  };
  const allCornersInside = (x, y, th) => cornersOf(x, y, th).every(p => p.x >= 0 && p.x <= W && p.y >= 0 && p.y <= H);
  function run(name, x, y, theta, keys, frames) {
    api.applyConfig(4, true); api.resetRound(true); api.startRound();
    api.setSpeedPct(100, true);
    S.car = { x: x, y: y, theta: theta, vl: 0, vr: 0 };
    api.clampCarByBounds(S.car);
    S.keys = keys; S.gpKeys = {};
    let outside = 0;
    for (let i = 0; i < frames; i++) { api.updateCar(1 / 60); if (!allCornersInside(S.car.x, S.car.y, S.car.theta)) outside++; }
    S.keys = {};
    ok('车不出界：' + name + '（' + frames + ' 帧，四角全程在场内）', outside === 0, '越界 ' + outside + ' 帧');
    return S.car;
  }
  // 1) 四边正顶 / 倒车顶墙：整车四角必须留在场内（θ=0 时车心 x 上限 = 600−85−0.5 = 514.5）
  const right = run('右墙正顶', 400, 200, 0, { up: true }, 400);
  ok('正顶右墙：车心停在 x≈514.5（整车贴边、四角不越界）',
    Math.abs(right.x - (W - 85 - MAR)) < 1.5 && allCornersInside(right.x, right.y, right.theta),
    'x=' + right.x.toFixed(2));
  const back = run('倒车顶右墙', 400, 200, Math.PI, { down: true }, 400);
  ok('倒车顶右墙：同样停在整车贴边位（x≈514.5），四角不越界',
    Math.abs(back.x - (W - 85 - MAR)) < 1.5 && allCornersInside(back.x, back.y, back.theta), 'x=' + back.x.toFixed(2));
  run('左墙正顶', 200, 200, Math.PI, { up: true }, 400);
  run('下墙正顶', 150, 700, Math.PI / 2, { up: true }, 400);
  run('上墙正顶', 150, 200, -Math.PI / 2, { up: true }, 400);
  // 2) 倒着顶左墙：θ=0 倒车向 −x，车心停在 95.5（整车贴左边，不是后轮贴边）
  const revLeft = run('倒车顶左墙', 300, 200, 0, { down: true }, 400);
  ok('倒车顶左墙：车心停在 x≈85.5（整车贴左边），四角不越界',
    Math.abs(revLeft.x - (85 + MAR)) < 2 && allCornersInside(revLeft.x, revLeft.y, revLeft.theta),
    'x=' + revLeft.x.toFixed(2));
  // 3) 斜着顶边：沿边滑行（另一轴仍有位移）
  const diag = run('斜顶右下角', 430, 700, Math.PI / 4, { up: true }, 300);
  ok('斜顶边时沿边滑行（另一轴仍有位移，不是被钉死）',
    Math.abs(diag.x - 430) > 1 || Math.abs(diag.y - 700) > 1);
  // 4) 贴墙原地转 / 边角边转边走：转向让四角扫出界时，车心被最小平移拉回（幂等、单帧不抖）
  api.applyConfig(4, true); api.resetRound(true); api.startRound();
  S.car = { x: W - 40, y: H / 2, theta: 0, vl: 0, vr: 0 };
  api.clampCarByBounds(S.car);
  S.keys = { left: true }; S.gpKeys = {};
  let rotOutside = 0, rotMono = true, rotMaxStep = 0, prevRotTheta = S.car.theta;
  for (let i = 0; i < 420; i++) {
    const px2 = S.car.x, py2 = S.car.y;
    api.updateCar(1 / 60);
    if (!inside0()) rotOutside++;
    if (S.car.theta < prevRotTheta - 1e-9) rotMono = false;
    prevRotTheta = S.car.theta;
    rotMaxStep = Math.max(rotMaxStep, Math.hypot(S.car.x - px2, S.car.y - py2));
  }
  S.keys = {};
  ok('贴右墙原地转 420 帧：四角全程在场内、theta 单调不抖、单帧最小平移 ≤8px',
    rotOutside === 0 && rotMono && rotMaxStep < 8, '越界 ' + rotOutside + ' maxStep=' + rotMaxStep.toFixed(2));
  api.applyConfig(4, true); api.resetRound(true); api.startRound();
  S.car = { x: W / 2, y: 160, theta: 0, vl: 0, vr: 0 };
  S.keys = { left: true, up: true }; S.gpKeys = {};
  let cornerOutside = 0;
  for (let i = 0; i < 420; i++) { api.updateCar(1 / 60); if (!allCornersInside(S.car.x, S.car.y, S.car.theta)) cornerOutside++; }
  S.keys = {};
  ok('边角边转边走 420 帧：四角全程在场内', cornerOutside === 0, '越界 ' + cornerOutside + ' 帧');
  // 5) 过桥入口整条路径
  {
    api.applyConfig(4, true); api.resetRound(true); api.startRound();
    const ang = api.bridgeAngle();
    const px = api.BRIDGE.cx + api.BRIDGE.dir[0] * -260;
    const py = api.BRIDGE.cy + api.BRIDGE.dir[1] * -260;
    S.car = { x: px, y: py, theta: ang, vl: 0, vr: 0 };
    S.keys = { up: true }; S.gpKeys = {};
    let bridgeOutside = 0;
    for (let i = 0; i < 1400; i++) { api.updateCar(1 / 60); if (!inside0()) bridgeOutside++; }
    S.keys = {};
    ok('从岸上开过桥 1400 帧：全程整车在场内', bridgeOutside === 0, '越界 ' + bridgeOutside + ' 帧');
  }
  // 6) 矩阵站位：所有可达站位必须满足"整车四角在场内（留 STATION_MARGIN）"
  {
    const rows = api.reachMatrix();
    const bad = rows.filter(r => r.reach && !api.carObbInside(r.station.x, r.station.y, r.station.theta, api.STATION_MARGIN - 1e-6))
      .map(r => r.from + '>' + r.to);
    ok('可达矩阵的所有可达站位都满足"整车四角在场内"（留 2px 余量）', bad.length === 0, bad.join(' '));
  }
  ok('页面写明车不出界口径（严格整车四角都在场内，车头/车尾不许越边）',
    html.includes('整车（170×100 四个角）都不会出场地') && html.includes('车头/车尾任何部分都不许越线'));
  // 7) 贴墙/角落原地转：每帧四角在场内、theta 单调（不抖）、单帧位移 ≤8px
  for (const [name, sx, sy] of [['贴右墙', W - 60, H / 2], ['左下角', 30, 30], ['右上角', W - 30, 30]]) {
    api.applyConfig(4, true); api.resetRound(true); api.startRound();
    S.car = { x: sx, y: sy, theta: 0, vl: 0, vr: 0 };
    api.clampCarByBounds(S.car);
    S.keys = { left: true }; S.gpKeys = {};
    let prevTheta = S.car.theta, mono = true, maxStep = 0, insideRot = true;
    for (let i = 0; i < 600; i++) {
      const px = S.car.x, py = S.car.y;
      api.updateCar(1 / 60);
      if (!allCornersInside(S.car.x, S.car.y, S.car.theta)) insideRot = false;
      if (S.car.theta < prevTheta - 1e-9) mono = false;
      prevTheta = S.car.theta;
      maxStep = Math.max(maxStep, Math.hypot(S.car.x - px, S.car.y - py));
    }
    S.keys = {};
    ok('原地转 600 帧（' + name + '）：四角每帧在场内、theta 单调不抖、单帧最小平移 ≤8px',
      insideRot && mono && maxStep < 8, 'maxStep=' + maxStep.toFixed(2));
  }
  // 8) 起始摆位合法化：5/6 号圈四种朝向 + 重置默认起点 4/5/6
  for (const id of [5, 6]) {
    for (const th of [0, Math.PI / 2, Math.PI, -Math.PI / 2]) {
      const c = { x: cRef(id).x, y: cRef(id).y, theta: th };
      api.clampCarByBounds(c);
      ok('起始摆位合法化：' + id + ' 号圈 θ=' + Math.round(th * 180 / Math.PI) + '° 四角在场内',
        api.carObbInside(c.x, c.y, c.theta, api.CAR_BOUND_MARGIN - 1e-6));
    }
  }
  for (const id of [4, 5, 6]) {
    api.applyConfig(id, true); api.resetRound(true);
    ok('重置到 ' + id + ' 号圈：起始位整车在场内', allCornersInside(S.car.x, S.car.y, S.car.theta));
  }
  // 9) 贴四条边发射：预判首落点 = 实际首落点（球出生点会被 traceShot clamp 到边框）
  {
    const edges = [
      ['左边', cRef(6).x, cRef(6).y, 0],
      ['上边', cRef(6).x, cRef(6).y, Math.PI / 2],
      ['右边', cRef(2).x, cRef(2).y, Math.PI],
      ['下边', cRef(2).x, cRef(2).y, -Math.PI / 2],
    ];
    for (const [name, x, y, th] of edges) {
      api.applyConfig(4, true); api.resetRound(true); api.startRound();
      S.car = { x: x, y: y, theta: th, vl: 0, vr: 0 };
      api.clampCarByBounds(S.car);
      const P = api.predictPoint(0, th, S.car.x, S.car.y);
      const b = { x: S.car.x, y: S.car.y, vx: 0, vy: 0, alive: true, flying: false, id: 0, handPlaced: false, originCircleId: null };
      S.balls = [b]; S.loadedBall = b; S.collectingBall = null; S.collectProgress = 2; S.scatter = false;
      api.tryFire(0); fly(b);
      ok('贴' + name + '发射：预判首落点 = 实际首落点（球出生点夹回场内）',
        Math.hypot(b.landing.x - P.x, b.landing.y - P.y) < 1e-9,
        'pred=(' + P.x.toFixed(1) + ',' + P.y.toFixed(1) + ') act=(' + b.landing.x.toFixed(1) + ',' + b.landing.y.toFixed(1) + ')');
    }
  }
  // 10) 面板"合法发射位"提示
  {
    api.applyConfig(4, true); api.resetRound(true); api.startRound();
    S.car = { x: cRef(4).x, y: cRef(4).y, theta: 0, vl: 0, vr: 0 };
    const a = api.fireSpotInfo();
    S.car = { x: 300, y: 450, theta: 0, vl: 0, vr: 0 };
    const b = api.fireSpotInfo();
    S.car = { x: 20, y: 450, theta: 0, vl: 0, vr: 0 };   // 整车越界（车头/车尾也出界）→ 不可发且提示差多少
    const c = api.fireSpotInfo();
    ok('面板合法发射位：在圈内→可发；圈外→显示差多少 px；整车越界也判不可发',
      a.ok === true && a.circleId === 4 && b.ok === false && b.gap > 0 && c.ok === false && c.gap > 0,
      'in=' + JSON.stringify(a) + ' out gap=' + b.gap.toFixed(1) + ' oob gap=' + c.gap.toFixed(1));
    ok('页面有"合法发射位"面板行', html.includes('id="fire-spot"') && html.includes('合法发射位'));
  }
  // 11) 贴边发射：球出生点夹回场内（BALL_R 边距），预判=实际
  {
    const cases = [
      ['左边', cRef(6), 0],
      ['右边', cRef(2), Math.PI],
      ['下边', cRef(2), -Math.PI / 2],
      ['上边', cRef(6), Math.PI / 2],
    ];
    for (const [name, C, th] of cases) {
      api.applyConfig(4, true); api.resetRound(true); api.startRound();
      S.car = { x: C.x, y: C.y, theta: th, vl: 0, vr: 0 };
      api.clampCarByBounds(S.car);
      const P = api.predictPoint(0, th, S.car.x, S.car.y);
      const b = { x: S.car.x, y: S.car.y, vx: 0, vy: 0, alive: true, flying: false, id: 0, handPlaced: false, originCircleId: null };
      S.balls = [b]; S.loadedBall = b; S.collectingBall = null; S.collectProgress = 2; S.scatter = false;
      api.tryFire(0); fly(b);
      const safe = P.spawn.x >= api.BALL_R - 1e-9 && P.spawn.x <= 600 - api.BALL_R + 1e-9 &&
        P.spawn.y >= api.BALL_R - 1e-9 && P.spawn.y <= 900 - api.BALL_R + 1e-9;
      ok('贴' + name + '发射：球出生点留 BALL_R 边距、预判首落点=实际',
        safe && Math.hypot(b.landing.x - P.x, b.landing.y - P.y) < 1e-9,
        'spawn=(' + P.spawn.x.toFixed(1) + ',' + P.spawn.y.toFixed(1) + ')');
    }
  }
  // 12) 严格口径 vs build-29 的"③ 车心+两后轮"：③ 下合法的旧标定位/旧站位现在整车越界，必须被夹回场内
  {
    const poses = [
      ['旧标定位（③ 下合法）', 307.98, 80.83, -113.37 * D2R],
      ['旧 5→1 站位（③ 下合法）', 50.3, 322.3, -152.9 * D2R],
      ['旧 5→3 站位（③ 下合法）', 73.8, 353.2, -49.8 * D2R],
    ];
    for (const [name, x, y, th] of poses) {
      const cornerOut = !allCornersInside(x, y, th);
      const c = { x: x, y: y, theta: th };
      api.clampCarByBounds(c);
      ok('严格口径：' + name + ' 车身角越界 → 被夹回整车合法位（build-29 ③ 已作废）',
        cornerOut && allCornersInside(c.x, c.y, c.theta) && api.carObbInside(c.x, c.y, c.theta, api.CAR_BOUND_MARGIN - 1e-6),
        'cornerOut=' + cornerOut);
    }
  }
  // 13) 判定函数与"实际夹取位姿"一致（防"只改判定、不改夹取"或反过来）
  {
    const W2 = api.ARENA_W, H2 = api.ARENA_H;
    const cases = [
      ['右墙 θ=0（车心越界 60）', W2 + 60, 400, 0, true],
      ['左墙 θ=0（车心越界 60）', -60, 400, 0, true],
      ['上墙 θ=0（车心越界 6）', 300, -6, 0, true],
      ['下墙 θ=0（车心越界 6）', 300, H2 + 6, 0, true],
      ['右墙斜放 45°（四角扫出）', W2, 500, Math.PI / 4, true],
      ['车心在内、四角扫出（45° 斜放）', 520, 520, Math.PI / 4, true],
      ['场内合法位（不应被夹）', 300, 450, 0.6, false],
    ];
    const bad = [];
    for (const [name, x, y, th, shouldBeOut] of cases) {
      const judgedBefore = api.carObbInside(x, y, th, api.CAR_BOUND_MARGIN);
      if (judgedBefore === shouldBeOut) bad.push(name + '(判定与几何事实不符：judged=' + judgedBefore + ')');
      const c = { x: x, y: y, theta: th };
      api.clampCarByBounds(c);
      const moved = Math.hypot(c.x - x, c.y - y);
      const judgedAfter = api.carObbInside(c.x, c.y, c.theta, api.CAR_BOUND_MARGIN - 1e-6);
      if (!judgedAfter || !allCornersInside(c.x, c.y, c.theta)) bad.push(name + '(夹取后仍非法)');
      if (shouldBeOut && moved < 1e-9) bad.push(name + '(判非法但夹取没动)');
      if (!shouldBeOut && moved > 1e-9) bad.push(name + '(判合法但夹取动了 ' + moved.toFixed(3) + 'px)');
    }
    ok('判定函数与实际夹取位姿一致：非法位姿夹取后必合法、合法位姿夹取不动（0 处不一致）',
      bad.length === 0, bad.join(' '));
    // 数值对账：θ=0 车体半长 85 → 右墙夹取后车心 x = 600−85−0.5 = 514.5
    const rc = { x: W2 + 60, y: 400, theta: 0 };
    api.clampCarByBounds(rc);
    const hh = api.carHalfExtent(0);
    ok('越界量 = 夹取位移量（右墙 θ=0：车心越界 60px，夹取后车心 x=514.5 = 600−85−0.5）',
      Math.abs(rc.x - (W2 - hh.hx - api.CAR_BOUND_MARGIN)) < 1e-9 && Math.abs(hh.hx - 85) < 1e-9,
      'center=' + rc.x.toFixed(3) + ' hx=' + hh.hx.toFixed(3));
  }
  api.setSpeedPct(50, true);          // 恢复默认，避免污染后面速度条断言
}

// ==================== 16. 底部控制区（build-12：摇杆 / A、B / 速度条）====================
{
  // 16.1 常量
  ok('控制区常量：摇杆 140/头 56/半径 42、最短按住 80ms、最小位移 8px（无幅值死区）、速度 20~70',
    api.JOY_SIZE === 140 && api.JOY_KNOB === 56 && Math.abs(api.JOY_R - 42) < 1e-9 &&
    api.JOY_MIN_HOLD_MS === 80 && api.JOY_MIN_DISP_PX === 8 && api.SPEED_DEAD === undefined &&
    api.SPEED_MIN === 20 && api.SPEED_MAX === 70);
  // 16.2 页面结构：左摇杆 + 右 A/B 圆钮 + 下方速度条；画布仍是 600×900
  ok('页面结构（用户现场确认）：左圆摇杆 + 右侧 A/B 两键水平并排 + 底部中间横拉条',
    html.includes('id="joy"') && html.includes('id="joy-knob"') &&
    html.includes('id="btn-fire"') && html.includes('A<small>发射</small>') &&
    html.includes('id="btn-collect"') && html.includes('B<small>收球</small>') &&
    html.includes('id="spd-range"') && !html.includes('rotate(-90deg)') &&
    html.includes('#deck-acts { display: flex; flex-direction: row;') &&
    html.includes('两键水平并排') &&
    html.includes('<canvas id="arena" width="600" height="900">'));
  ok('控制区不影响画布尺寸（canvas 仍 600×900，速度条不会挤压地图）',
    /<canvas id="arena" width="600" height="900">/.test(html) && html.includes('#deck') );
  ok('页面写了摇杆"轻扫不动车"提示（<80ms 或 <8px 过滤）', html.includes('轻扫（&lt;80ms）或位移 &lt;8px 不会动'));
  ok('A/B 已按用户确认（A=发射 / B=收球），页面注明官方 App 无收球键',
    html.includes('A=发射') && html.includes('B=收球') &&
    html.includes('官方 App 无收球键') && !html.includes('待用户现场确认'));
  ok('A、B 水平并排（解析真实 CSS 规则：#deck-acts 的 flex-direction = row，且无 column 残留）', (() => {
    const m = html.match(/#deck-acts\s*\{([^}]*)\}/);
    if (!m) return false;
    const block = m[1];
    return /flex-direction:\s*row/.test(block) && !/flex-direction:\s*column/.test(block) &&
      /display:\s*flex/.test(block);
  })());
  ok('页面已删“待用户现场确认”（用户已确认 A=发射/B=收球）',
    !html.includes('待用户现场确认') && html.includes('已按用户现场确认'));
  ok('控制区尺寸沿用官方比例：摇杆 220rpx→140px、头 68rpx→56px、按钮 60rpx→64px；拉条为横向',
    html.includes('width: 140px; height: 140px') && html.includes('56px') &&
    html.includes('width: 64px; height: 64px') &&
    html.includes('#spd input[type=range] { flex: 1;') && !html.includes('#spd-track'));
  ok('摇杆手感按 APK：按下记起点（delta），不是按下即跳位',
    html.includes('state.joy.sx') && html.includes('clientX - state.joy.sx') &&
    html.includes('APK：按下瞬间摇杆回中，不跳位'));
  // 16.3 速度条：0~100 → 20~70 px/s，键盘和摇杆都受它限制
  ok('速度条连续 0~100、默认 50%（页面 step=1、value=50）',
    html.includes('id="spd-range" min="0" max="100" step="1" value="50"') && S.speedPct === 50);
  ok('速度条默认 50% → 最高车速 45 px/s', Math.abs(api.maxCarSpeed() - 45) < 1e-9, api.maxCarSpeed().toFixed(1));
  api.setSpeedPct(0, true);
  ok('速度条 0% → 20 px/s', Math.abs(api.maxCarSpeed() - 20) < 1e-9);
  api.setSpeedPct(100, true);
  ok('速度条 100% → 70 px/s', Math.abs(api.maxCarSpeed() - 70) < 1e-9);

  // 16.4 轻扫（按住 <150ms）车不动；按住够时长才走；松手立刻停
  function joySetup() {
    api.applyConfig(4, true); api.resetRound(true); api.startRound();
    api.setSpeedPct(100, true);
    S.car = { x: 300, y: 200, theta: 0, vl: 0, vr: 0 };
    S.keys = {}; S.gpKeys = {};
    api.releaseJoy();
  }
  function runCar(frames) { for (let i = 0; i < frames; i++) api.updateCar(1 / 60); }
  joySetup();
  S.joy.down = true; S.joy.ox = 0; S.joy.oy = -api.JOY_R;
  runCar(4);            // 66ms < 80ms 最短按住
  const flickDx = S.car.x - 300;
  api.releaseJoy(); runCar(6);
  ok('轻扫一下（按住 66ms < 80ms）车不动（闪一下不给位移）', Math.abs(flickDx) < 1e-9, 'dx=' + flickDx.toFixed(4) + 'px');
  joySetup();
  S.joy.down = true; S.joy.ox = 0; S.joy.oy = -api.JOY_R;
  runCar(5);            // 83ms > 80ms
  const hold83 = S.car.x - 300;
  api.releaseJoy();
  ok('按住 83ms（过 80ms 门限）轻微位移 > 0', hold83 > 0.01, 'dx=' + hold83.toFixed(3));

  joySetup();
  S.joy.down = true; S.joy.ox = 0; S.joy.oy = -api.JOY_R;
  runCar(12);           // 200ms：越过 80ms 门限后开始走
  const holdDx = S.car.x - 300;
  ok('持续按住并推出死区后车才走（200ms 时已有位移）', holdDx > 1, 'dx=' + holdDx.toFixed(2) + 'px');
  api.releaseJoy();
  const xAfterStop = S.car.x;
  runCar(10);
  ok('松手立刻停（松手后 10 帧位移为 0）', Math.abs(S.car.x - xAfterStop) < 1e-9, 'dx=' + (S.car.x - xAfterStop).toFixed(4));

  // 16.5 最小位移过滤（不是幅值死区）：位移 <8px 不动，≥8px 能动（哪怕速度只有 ~20）
  joySetup();
  S.joy.down = true; S.joy.ox = 0; S.joy.oy = -4;      // 位移 4px < 8px
  runCar(40);
  ok('摇杆位移 4px（<8px）车不动', Math.abs(S.car.x - 300) < 1e-9 && Math.abs(S.car.y - 200) < 1e-9,
    'disp=' + api.joyMotion().dispPx.toFixed(1) + 'px');
  api.releaseJoy();
  joySetup();
  S.joy.down = true; S.joy.ox = 0; S.joy.oy = -10;     // 位移 10px ≥ 8px，速度约 24（<30 也照样低速走）
  runCar(40);
  ok('摇杆位移 10px（≥8px）能动，且低速对应小速度（无 <30 幅值死区）', S.car.x > 300 + 0.5,
    'disp=' + api.joyMotion().dispPx.toFixed(1) + 'px spd=' + api.joyMotion().spd + ' dx=' + (S.car.x - 300).toFixed(2));
  api.releaseJoy();

  // 16.6 方向：上推前进、右推右转（theta 减小）、左推左转（theta 增大）
  // 注：幅度越大速度越大（APK：幅度即速度）
  joySetup();
  S.joy.down = true; S.joy.ox = 0; S.joy.oy = -api.JOY_R;
  runCar(30);
  ok('摇杆上推 = 前进（沿 theta 正向）', S.car.x > 300 + 5 && Math.abs(S.car.y - 200) < 1e-6, 'dx=' + (S.car.x - 300).toFixed(1));
  api.releaseJoy();
  joySetup();
  S.joy.down = true; S.joy.ox = api.JOY_R; S.joy.oy = 0;
  runCar(30);
  ok('摇杆右推 = 原地右转（theta 减小、位置不动）', S.car.theta < -0.2 && Math.abs(S.car.x - 300) < 1e-9, 'dθ=' + S.car.theta.toFixed(2));
  api.releaseJoy();
  joySetup();
  S.joy.down = true; S.joy.ox = -api.JOY_R; S.joy.oy = 0;
  runCar(30);
  ok('摇杆左推 = 原地左转（theta 增大）', S.car.theta > 0.2, 'dθ=' + S.car.theta.toFixed(2));
  api.releaseJoy();

  // 16.7 速度条真的改车速：同样的按住时长，0% 与 100% 位移比 ≈ 20/70
  function joyRunDx(pct) {
    joySetup(); api.setSpeedPct(pct, true);
    S.joy.down = true; S.joy.ox = 0; S.joy.oy = -api.JOY_R;
    runCar(40);
    const dx = S.car.x - 300;
    api.releaseJoy();
    return dx;
  }
  const dx0 = joyRunDx(0), dx100 = joyRunDx(100);
  // 前 150ms 门限内不走，两档位移都从同一时刻开始累积，比值应≈20/70
  ok('速度条 0% 明显比 100% 慢（位移比≈20/70，实测 ' + (dx0 / dx100).toFixed(2) + '）',
    dx100 > 10 && dx0 > 2 && dx0 < dx100 * 0.45);
  // 16.8 键盘也受速度条限制
  joySetup(); api.setSpeedPct(100, true);
  press('ArrowUp'); frames(10, 16); release('ArrowUp'); frames(1, 16);
  const kdx100 = S.car.x - 300;
  joySetup(); api.setSpeedPct(0, true);
  press('ArrowUp'); frames(10, 16); release('ArrowUp'); frames(1, 16);
  const kdx0 = S.car.x - 300;
  ok('键盘按 ↑ 也受速度条限制（0% 比 100% 慢，实测比 ' + (kdx0 / kdx100).toFixed(2) + '）',
    kdx100 > 5 && kdx0 < kdx100 * 0.6);
  api.setSpeedPct(50, true);
}

// ==================== 16. build-31：指挥部 140 + 撞护栏 60 帧不减速 ====================
{
  ok('指挥部：判定半径 = 绘制半径 = 140（绘制直接用 base.r，BASE_DRAW_R 第二来源已删）',
    api.BASE_US.r === 140 && api.BASE_THEM.r === 140 &&
    html.includes('ctx.arc(base.x, base.y, base.r') && !html.includes('BASE_DRAW_R'));
  ok('常量清理：旧 CAR_R / BRIDGE_RAIL.inset 已删除（无第二来源误导）',
    !html.includes('CAR_R = 55') && !html.includes('BRIDGE_RAIL.inset') && !html.includes('inset: 6'));
  // 到点判定：车心距对方指挥部 130 → 到达；150 → 未到达
  api.applyConfig(4, true); api.resetRound(true); api.startRound();
  S.car = { x: 91.92, y: 808.08, theta: 0, vl: 0, vr: 0 };   // 距 (0,900) = 130
  S.keys = {}; api.updateCar(1 / 60);
  ok('到达判定：车心距对方指挥部 130px（<140）→ 判定到达并结束', S.reachedHQ === true && S.phase === 'finished');
  api.applyConfig(4, true); api.resetRound(true); api.startRound();
  S.car = { x: 106.07, y: 793.93, theta: 0, vl: 0, vr: 0 };  // 距 (0,900) = 150
  S.keys = {}; api.updateCar(1 / 60);
  ok('到达判定：车心距对方指挥部 150px（>140）→ 未到达（140 不是虚设）', S.reachedHQ === false && S.phase !== 'finished');
  // 撞护栏连续 60 帧：与不撞护栏的位移/朝向逐帧一致
  function runRailDrive(p) {
    api.applyConfig(4, true); api.resetRound(true); api.startRound();
    S.lastBarrierAt = -9999;
    const angD = api.bridgeAngle();
    const bx = api.BRIDGE.cx + api.BRIDGE.dir[0] * -20 + (-api.BRIDGE.dir[1]) * p;
    const by = api.BRIDGE.cy + api.BRIDGE.dir[1] * -20 + (api.BRIDGE.dir[0]) * p;
    S.car = { x: bx, y: by, theta: angD, vl: 0, vr: 0 };
    S.keys = { up: true }; S.gpKeys = {};
    const x0 = S.car.x, y0 = S.car.y;
    for (let i = 0; i < 60; i++) api.updateCar(1 / 60);
    S.keys = {};
    return { dx: S.car.x - x0, dy: S.car.y - y0, th: S.car.theta, hits: S.barrierHits };
  }
  const rc0 = runRailDrive(0), rc1 = runRailDrive(-25);
  ok('撞护栏连续 60 帧：位移与朝向与不撞时完全一致（不减速、不偏航）',
    rc1.hits > 0 && rc0.hits === 0 &&
    Math.abs(rc0.dx - rc1.dx) < 1e-9 && Math.abs(rc0.dy - rc1.dy) < 1e-9 && Math.abs(rc0.th - rc1.th) < 1e-9,
    'hits=' + rc1.hits + ' d=' + Math.hypot(rc1.dx - rc0.dx, rc1.dy - rc0.dy).toExponential(2));
  // barrierHits 的语义 = 节流后的事件次数（不是逐帧碰撞帧数）
  {
    api.applyConfig(4, true); api.resetRound(true); api.startRound();
    S.lastBarrierAt = -9999;
    const angS = api.bridgeAngle();
    const x0 = api.BRIDGE.cx + api.BRIDGE.dir[0] * -60 + (-api.BRIDGE.dir[1]) * 25;
    const y0 = api.BRIDGE.cy + api.BRIDGE.dir[1] * -60 + (api.BRIDGE.dir[0]) * 25;
    S.car = { x: x0, y: y0, theta: angS, vl: 0, vr: 0 };
    S.keys = { up: true }; S.gpKeys = {};
    let scraped = 0;
    for (let i = 0; i < 180; i++) { api.updateCar(1 / 60); if (api.carRailHit(S.car.x, S.car.y, S.car.theta)) scraped++; }
    S.keys = {};
    ok('barrierHits 是"节流后的事件次数"：持续擦护栏 3 秒只 +1~2（不是按帧 +180）',
      scraped > 60 && S.barrierHits >= 1 && S.barrierHits <= 2,
      'scrapedFrames=' + scraped + ' hits=' + S.barrierHits);
  }
  // 入口判定必须是"真实车体矩形 × 挡板矩形相交"，不是旧的车体半长外扩（AABB 近似会误报）。
  // 旧近似：halfT/halfP 取车体在桥轴/垂直轴上的半投影，再拿 AABB 与挡板带比较；
  // 斜着靠近水面段入口时 AABB 会先碰到挡板带，而真实 OBB 还没碰到。
  {
    const dir = api.BRIDGE.dir, px = -dir[1], py = dir[0];
    const off = api.BRIDGE_RAIL.centerOffset, thick = api.BRIDGE_RAIL.thick;
    const span1 = api.RAIL_SPAN[1];
    const relOf = th => th - api.bridgeAngle();
    function heuristic(x, y, th) {
      const rel = relOf(th), ca = Math.cos(rel), sa = Math.sin(rel);
      const halfT = Math.abs(ca) * api.CAR_LEN / 2 + Math.abs(sa) * api.CAR_WID / 2;
      const halfP = Math.abs(sa) * api.CAR_LEN / 2 + Math.abs(ca) * api.CAR_WID / 2;
      const q = api.bridgeLocal(x, y);
      for (const s of [1, -1]) {
        const sp = api.RAIL_SPAN[s];
        if (q.t + halfT < sp[0] || q.t - halfT > sp[1]) continue;
        if (Math.abs(q.p - s * off) <= thick / 2 + halfP) return s;
      }
      return 0;
    }
    let falsePos = null;
    for (let t = -190; t <= 190 && !falsePos; t += 2) {
      for (let p = -120; p <= 120 && !falsePos; p += 2) {
        for (let d = -89; d <= 89 && !falsePos; d += 1) {
          const th = api.bridgeAngle() + d * Math.PI / 180;
          const x = api.BRIDGE.cx + t * dir[0] + p * px;
          const y = api.BRIDGE.cy + t * dir[1] + p * py;
          const qc = api.bridgeLocal(x, y);
          if (qc.t < span1[0] || qc.t > span1[1]) continue;   // 只比"车心已在水面段"的位姿
          if (heuristic(x, y, th) !== 0 && api.carRailHit(x, y, th) === 0) falsePos = { t, p, d };
        }
      }
    }
    ok('入口判定用真实矩形相交：存在"旧 AABB 近似会误报、真实 OBB 不相交"的位姿（旧实现会红）',
      !!falsePos, falsePos ? 't=' + falsePos.t + ' p=' + falsePos.p + ' 偏' + falsePos.d + '°' : '未找到（说明还是 AABB 近似）');
    if (falsePos) {
      const th = api.bridgeAngle() + falsePos.d * Math.PI / 180;
      const x = api.BRIDGE.cx + falsePos.t * dir[0] + falsePos.p * px;
      const y = api.BRIDGE.cy + falsePos.t * dir[1] + falsePos.p * py;
      ok('该位姿下真实车体矩形与挡板不相交（carRailHit=0），不再误报撞护栏',
        api.carRailHit(x, y, th) === 0 && heuristic(x, y, th) !== 0);
    }
    // 用户 2026-10-18 明确："只要车体碰到挡板就报警"，不再要求车心进入水面段。
    // 构造：车心还在岸上段、车头伸进水面段、车体横向压到挡板带 → 必须报；车心进水面段后同一横向位置 → 也报。
    {
      const thB = api.bridgeAngle();
      const tOnShore = span1[0] - 50;          // 车心在岸上，距水面段入口 50px
      const xOn = api.BRIDGE.cx + tOnShore * dir[0] + 25 * px;
      const yOn = api.BRIDGE.cy + tOnShore * dir[1] + 25 * py;
      const tInWater = span1[0] + 10;          // 车心已进水面段 10px
      const xIn = api.BRIDGE.cx + tInWater * dir[0] + 25 * px;
      const yIn = api.BRIDGE.cy + tInWater * dir[1] + 25 * py;
      ok('只要碰到就报：车心还在岸上段（车头伸进水面段、横向压到挡板）→ 报；车心进水面段后 → 也报',
        api.bridgeLocal(xOn, yOn).t < span1[0] && api.carRailHit(xOn, yOn, thB) !== 0 &&
        api.bridgeLocal(xIn, yIn).t > span1[0] && api.carRailHit(xIn, yIn, thB) !== 0);
    }
    // 反向：真实相交的位姿必须能判出来（p=0 正对桥心、车心在水面段中，两侧都压不到；p=25 才压到）
    const tMid = (span1[0] + span1[1]) / 2;
    const atP = pp => {
      const x = api.BRIDGE.cx + tMid * dir[0] + pp * px;
      const y = api.BRIDGE.cy + tMid * dir[1] + pp * py;
      return api.carRailHit(x, y, api.bridgeAngle());
    };
    ok('真实相交：p=0 正对桥心不撞护栏、p=30 压到挡板判出（净宽 134、车宽 100）',
      atP(0) === 0 && atP(30) !== 0, 'p0=' + atP(0) + ' p30=' + atP(30));
  }
}

console.log('\n' + pass + ' passed / ' + fail + ' failed');
process.exit(fail ? 1 : 0);
