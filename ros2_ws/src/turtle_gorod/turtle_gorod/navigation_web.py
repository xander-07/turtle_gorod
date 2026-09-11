#!/usr/bin/env python3
import base64
import json
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


HTML = r"""<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>turtle_gorod — навигация</title>
<style>
  html,body{margin:0;width:100%;height:100%;overflow:hidden;background:#202124;color:#eee;font-family:Arial,sans-serif}
  #bar{position:absolute;z-index:2;left:12px;top:12px;background:rgba(20,20,20,.90);padding:10px 12px;border-radius:8px;font-size:14px;line-height:1.45;max-width:760px}
  #status{font-weight:700}
  #navstatus{color:#9be564;margin-top:3px}
  #hint{color:#ffd166;margin-top:4px}
  #buttons{display:flex;gap:7px;flex-wrap:wrap;margin-top:8px}
  button{border:1px solid #555;background:#2d2f31;color:#eee;padding:7px 10px;border-radius:6px;cursor:pointer;font-weight:600}
  button.active{outline:2px solid #20d5e8;border-color:#20d5e8}
  button.stop{background:#7b1e1e;border-color:#a33}
  canvas{display:block;width:100vw;height:100vh;cursor:crosshair}
</style>
</head>
<body>
<div id="bar">
  <div id="status">Ожидание карты…</div>
  <div id="info"></div>
  <div id="navstatus"></div>
  <div>чёрное — препятствия · красное — LiDAR · голубое — робот · зелёное — маршрут · фиолетовое — цель</div>
  <div id="hint">Сначала задайте начальную позицию робота.</div>
  <div id="buttons">
    <button id="initialBtn">Начальная позиция</button>
    <button id="goalBtn">Цель Nav2</button>
    <button id="cancelBtn" class="stop">СТОП / отменить цель</button>
  </div>
</div>
<canvas id="c"></canvas>
<script>
const canvas=document.getElementById('c');
const ctx=canvas.getContext('2d');
const statusEl=document.getElementById('status');
const infoEl=document.getElementById('info');
const navStatusEl=document.getElementById('navstatus');
const hintEl=document.getElementById('hint');
const initialBtn=document.getElementById('initialBtn');
const goalBtn=document.getElementById('goalBtn');
const cancelBtn=document.getElementById('cancelBtn');

let map=null, pose=null, scan=[], plan=[];
let goal=null;
let off=document.createElement('canvas'), offctx=off.getContext('2d');
let currentRev=-1;
let view=null;
let dragStart=null, dragNow=null;
let initialMarker=null;
let mode='initial';
let autoSwitched=false;

function setMode(m){
  mode=m;
  initialBtn.classList.toggle('active',mode==='initial');
  goalBtn.classList.toggle('active',mode==='goal');
  if(mode==='initial'){
    hintEl.textContent='Зажмите ЛКМ на реальном положении робота и потяните в направлении его передней части.';
  }else{
    hintEl.textContent='Кликните точку назначения. Или зажмите ЛКМ и протяните стрелку, чтобы точно задать конечное направление.';
  }
}
initialBtn.onclick=()=>setMode('initial');
goalBtn.onclick=()=>setMode('goal');
cancelBtn.onclick=async()=>{
  try{
    await fetch('/cancel',{method:'POST'});
    navStatusEl.textContent='Отмена цели отправлена…';
  }catch(e){ navStatusEl.textContent='Не удалось отправить отмену'; }
};

function resize(){
  const dpr=window.devicePixelRatio||1;
  canvas.width=Math.floor(innerWidth*dpr); canvas.height=Math.floor(innerHeight*dpr);
  ctx.setTransform(dpr,0,0,dpr,0,0);
  draw();
}
window.addEventListener('resize',resize);

function decodeMap(b64,w,h){
  const raw=atob(b64), arr=new Uint8ClampedArray(w*h*4);
  for(let i=0;i<raw.length;i++){
    const v=raw.charCodeAt(i), j=i*4;
    let c=70;
    if(v===1)c=235; else if(v===2)c=10;
    arr[j]=c; arr[j+1]=c; arr[j+2]=c; arr[j+3]=255;
  }
  off.width=w; off.height=h;
  offctx.putImageData(new ImageData(arr,w,h),0,0);
}

function worldToGrid(x,y){
  const dx=x-map.ox, dy=y-map.oy, c=Math.cos(map.oyaw), s=Math.sin(map.oyaw);
  return [(c*dx+s*dy)/map.res,(-s*dx+c*dy)/map.res];
}

function gridToWorld(gx,gy){
  const c=Math.cos(map.oyaw), s=Math.sin(map.oyaw);
  return [map.ox+(c*gx-s*gy)*map.res,map.oy+(s*gx+c*gy)*map.res];
}

function canvasToWorld(cx,cy){
  if(!map || !view)return null;
  const gx=(cx-view.ox)/view.scale;
  const gy=(view.oy+view.dh-cy)/view.scale;
  if(gx<0 || gy<0 || gx>map.w || gy>map.h)return null;
  return gridToWorld(gx,gy);
}

function worldToCanvas(x,y){
  if(!map || !view)return null;
  const g=worldToGrid(x,y);
  return [view.ox+g[0]*view.scale,view.oy+view.dh-g[1]*view.scale];
}

function drawArrow(x,y,yaw,color,size=13){
  const q=worldToCanvas(x,y);
  if(!q)return;
  ctx.save();
  ctx.translate(q[0],q[1]);
  ctx.rotate(-yaw);
  ctx.fillStyle=color;
  ctx.beginPath();
  ctx.moveTo(size,0);
  ctx.lineTo(-0.7*size,-0.6*size);
  ctx.lineTo(-0.7*size,0.6*size);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function draw(){
  ctx.setTransform(1,0,0,1,0,0);
  ctx.fillStyle='#202124'; ctx.fillRect(0,0,canvas.width,canvas.height);
  const dpr=window.devicePixelRatio||1, W=canvas.width/dpr, H=canvas.height/dpr;
  ctx.setTransform(dpr,0,0,dpr,0,0);
  if(!map){view=null; return;}

  const margin=28, scale=Math.min((W-2*margin)/map.w,(H-2*margin)/map.h);
  const dw=map.w*scale, dh=map.h*scale, ox=(W-dw)/2, oy=(H-dh)/2;
  view={scale,dw,dh,ox,oy};

  ctx.imageSmoothingEnabled=false;
  ctx.save();
  ctx.translate(ox,oy+dh); ctx.scale(1,-1);
  ctx.drawImage(off,0,0,dw,dh);
  ctx.restore();

  if(plan.length>1){
    ctx.strokeStyle='#7CFC00';
    ctx.lineWidth=3;
    ctx.beginPath();
    let first=true;
    for(const p of plan){
      const q=worldToCanvas(p[0],p[1]);
      if(!q)continue;
      if(first){ctx.moveTo(q[0],q[1]); first=false;}else ctx.lineTo(q[0],q[1]);
    }
    ctx.stroke();
  }

  ctx.fillStyle='#ff3b30';
  for(const p of scan){
    const q=worldToCanvas(p[0],p[1]);
    if(q)ctx.fillRect(q[0]-1.3,q[1]-1.3,2.6,2.6);
  }

  if(initialMarker)drawArrow(initialMarker.x,initialMarker.y,initialMarker.yaw,'#ffb000',15);
  if(goal)drawArrow(goal.x,goal.y,goal.yaw,'#d946ef',16);
  if(pose)drawArrow(pose.x,pose.y,pose.yaw,'#20d5e8',13);

  if(dragStart && dragNow){
    ctx.strokeStyle=mode==='goal'?'#d946ef':'#ffb000';
    ctx.lineWidth=3;
    ctx.beginPath();
    ctx.moveTo(dragStart.cx,dragStart.cy);
    ctx.lineTo(dragNow.cx,dragNow.cy);
    ctx.stroke();
  }
}

async function postPose(path,x,y,yaw){
  const r=await fetch(path,{
    method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({x,y,yaw}),
  });
  if(!r.ok)throw new Error('HTTP '+r.status);
}

async function sendInitialPose(x,y,yaw){
  try{
    await postPose('/initialpose',x,y,yaw);
    initialMarker={x,y,yaw};
    statusEl.textContent='Начальная позиция отправлена в AMCL';
    hintEl.textContent='Подождите несколько секунд: LiDAR должен совместиться со стенами карты.';
    draw();
  }catch(e){ statusEl.textContent='Не удалось отправить initial pose'; }
}

async function sendGoal(x,y,yaw){
  try{
    await postPose('/goal',x,y,yaw);
    goal={x,y,yaw};
    navStatusEl.textContent='Цель отправлена в Nav2…';
    draw();
  }catch(e){ navStatusEl.textContent='Не удалось отправить цель Nav2'; }
}

canvas.addEventListener('mousedown',e=>{
  if(e.button!==0 || !map || !view)return;
  const rect=canvas.getBoundingClientRect();
  const cx=e.clientX-rect.left, cy=e.clientY-rect.top;
  const w=canvasToWorld(cx,cy);
  if(!w)return;
  dragStart={cx,cy,x:w[0],y:w[1]};
  dragNow={cx,cy};
  draw();
});

canvas.addEventListener('mousemove',e=>{
  if(!dragStart)return;
  const rect=canvas.getBoundingClientRect();
  dragNow={cx:e.clientX-rect.left,cy:e.clientY-rect.top};
  draw();
});

window.addEventListener('mouseup',e=>{
  if(e.button!==0 || !dragStart)return;
  const rect=canvas.getBoundingClientRect();
  const cx=e.clientX-rect.left, cy=e.clientY-rect.top;
  const start=dragStart;
  const pixels=Math.hypot(cx-start.cx,cy-start.cy);
  dragStart=null; dragNow=null;
  const end=canvasToWorld(cx,cy);
  if(!end){draw(); return;}

  if(mode==='initial'){
    if(pixels<8){
      statusEl.textContent='Для initial pose нужно указать направление';
      hintEl.textContent='Зажмите ЛКМ на позиции робота и протяните стрелку вперёд по направлению корпуса.';
      draw(); return;
    }
    const yaw=Math.atan2(end[1]-start.y,end[0]-start.x);
    sendInitialPose(start.x,start.y,yaw);
    return;
  }

  let yaw;
  if(pixels<8){
    if(pose)yaw=Math.atan2(start.y-pose.y,start.x-pose.x);
    else yaw=0.0;
    sendGoal(start.x,start.y,yaw);
  }else{
    yaw=Math.atan2(end[1]-start.y,end[0]-start.x);
    sendGoal(start.x,start.y,yaw);
  }
});

async function tick(){
  try{
    const r=await fetch('/state?map_rev='+currentRev,{cache:'no-store'});
    if(!r.ok)throw new Error('HTTP '+r.status);
    const s=await r.json();
    if(s.map){map=s.map; currentRev=map.rev; decodeMap(map.data,map.w,map.h);}
    pose=s.pose;
    scan=s.scan||[];
    plan=s.plan||[];
    goal=s.goal||goal;

    if(map && !pose)statusEl.textContent='Карта получена · AMCL ждёт начальную позицию';
    else if(map && pose)statusEl.textContent='Локализация активна';
    else if(!map)statusEl.textContent='Ожидание /map…';

    if(pose && !autoSwitched){
      autoSwitched=true;
      setMode('goal');
    }

    if(map){
      const p=pose?` · робот x=${pose.x.toFixed(2)} y=${pose.y.toFixed(2)} yaw=${(pose.yaw*180/Math.PI).toFixed(1)}°`:'';
      infoEl.textContent=`${map.w}×${map.h} · ${map.res.toFixed(3)} м/ячейку${p}`;
    }

    let ns=s.nav_status||'';
    if(s.distance_remaining!==null && s.distance_remaining!==undefined && ns==='Еду к цели'){
      ns+=` · осталось ${Number(s.distance_remaining).toFixed(2)} м`;
    }
    navStatusEl.textContent=ns;
    draw();
  }catch(e){ statusEl.textContent='Нет связи с роботом'; }
  setTimeout(tick,200);
}
setMode('initial');
resize(); tick();
</script>
</body>
</html>"""


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


class NavigationWebNode(Node):
    def __init__(self):
        super().__init__("navigation_web")
        self.declare_parameter("port", 8081)
        self.port = int(self.get_parameter("port").value)

        self.lock = threading.Lock()
        self.map_revision = 0
        self.map_payload = None
        self.pose = None
        self.scan_points = []
        self.plan_points = []
        self.goal_payload = None
        self.nav_status = "Nav2 готов"
        self.distance_remaining = None
        self.pending_initial_pose = None
        self.pending_goal = None
        self.pending_cancel = False
        self.goal_handle = None
        self.goal_seq = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self.create_timer(0.10, self._update_pose)
        self.create_timer(0.05, self._process_requests)

        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                return

            def _send_json(self, status, payload):
                body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _read_pose(self):
                length = int(self.headers.get("Content-Length", "0"))
                data = json.loads(self.rfile.read(length).decode("utf-8"))
                x = float(data["x"])
                y = float(data["y"])
                yaw = float(data["yaw"])
                if not all(math.isfinite(v) for v in (x, y, yaw)):
                    raise ValueError("non-finite pose")
                return x, y, yaw

            def do_GET(self):
                parsed = urlparse(self.path)
                if parsed.path == "/":
                    body = HTML.encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if parsed.path == "/state":
                    qs = parse_qs(parsed.query)
                    try:
                        client_rev = int(qs.get("map_rev", ["-1"])[0])
                    except ValueError:
                        client_rev = -1
                    with node.lock:
                        payload = {
                            "pose": node.pose,
                            "scan": list(node.scan_points),
                            "plan": list(node.plan_points),
                            "goal": node.goal_payload,
                            "nav_status": node.nav_status,
                            "distance_remaining": node.distance_remaining,
                            "map": node.map_payload if client_rev != node.map_revision else None,
                        }
                    self._send_json(200, payload)
                    return

                self.send_error(404)

            def do_POST(self):
                parsed = urlparse(self.path)
                try:
                    if parsed.path == "/initialpose":
                        pose = self._read_pose()
                        with node.lock:
                            node.pending_initial_pose = pose
                        self._send_json(200, {"ok": True})
                        return

                    if parsed.path == "/goal":
                        pose = self._read_pose()
                        with node.lock:
                            node.pending_goal = pose
                            node.nav_status = "Цель принята веб-интерфейсом"
                        self._send_json(200, {"ok": True})
                        return

                    if parsed.path == "/cancel":
                        with node.lock:
                            node.pending_goal = None
                            node.pending_cancel = True
                            node.nav_status = "Отменяю цель…"
                        self._send_json(200, {"ok": True})
                        return

                    self.send_error(404)
                except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
                    self._send_json(400, {"ok": False, "error": str(exc)})

        self.httpd = ReusableThreadingHTTPServer(("0.0.0.0", self.port), Handler)
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(
            f"Navigation web: http://0.0.0.0:{self.port} "
            "(initial pose, Nav2 goal, route and cancel)"
        )

    def _on_map(self, msg):
        packed = bytearray(len(msg.data))
        for i, value in enumerate(msg.data):
            if value < 0:
                packed[i] = 0
            elif value >= 50:
                packed[i] = 2
            else:
                packed[i] = 1

        self.map_revision += 1
        payload = {
            "rev": self.map_revision,
            "w": int(msg.info.width),
            "h": int(msg.info.height),
            "res": float(msg.info.resolution),
            "ox": float(msg.info.origin.position.x),
            "oy": float(msg.info.origin.position.y),
            "oyaw": yaw_from_quaternion(msg.info.origin.orientation),
            "data": base64.b64encode(bytes(packed)).decode("ascii"),
        }
        with self.lock:
            self.map_payload = payload

    def _on_plan(self, msg):
        poses = msg.poses
        step = max(1, len(poses) // 500)
        points = [
            [round(p.pose.position.x, 3), round(p.pose.position.y, 3)]
            for p in poses[::step]
        ]
        with self.lock:
            self.plan_points = points

    def _update_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            pose = {"x": float(t.x), "y": float(t.y), "yaw": yaw_from_quaternion(q)}
            with self.lock:
                self.pose = pose
        except Exception:
            pass

    def _on_scan(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform("map", msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return

        t = tf.transform.translation
        yaw = yaw_from_quaternion(tf.transform.rotation)
        cy, sy = math.cos(yaw), math.sin(yaw)
        points = []
        step = max(1, len(msg.ranges) // 400)
        for i in range(0, len(msg.ranges), step):
            rng = msg.ranges[i]
            if not math.isfinite(rng) or rng < msg.range_min or rng > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            lx, ly = rng * math.cos(angle), rng * math.sin(angle)
            mx = t.x + cy * lx - sy * ly
            my = t.y + sy * lx + cy * ly
            points.append([round(mx, 3), round(my, 3)])

        with self.lock:
            self.scan_points = points

    def _process_requests(self):
        with self.lock:
            initial = self.pending_initial_pose
            self.pending_initial_pose = None
            goal = self.pending_goal
            self.pending_goal = None
            cancel = self.pending_cancel
            self.pending_cancel = False

        if initial is not None:
            self._publish_initial_pose(initial)

        if cancel:
            self._cancel_goal()

        if goal is not None:
            if not self.nav_client.server_is_ready():
                with self.lock:
                    self.pending_goal = goal
                    self.nav_status = "Жду action-сервер Nav2…"
                return
            self._send_goal(goal)

    def _publish_initial_pose(self, pose):
        x, y, yaw = pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
        msg.pose.covariance[0] = 0.25 * 0.25
        msg.pose.covariance[7] = 0.25 * 0.25
        msg.pose.covariance[35] = math.radians(15.0) ** 2
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(
            f"Initial pose sent: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f} deg"
        )

    def _send_goal(self, pose):
        x, y, yaw = pose
        self.goal_seq += 1
        seq = self.goal_seq

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal_msg.pose.pose.orientation.w = math.cos(yaw * 0.5)

        with self.lock:
            self.goal_payload = {"x": x, "y": y, "yaw": yaw}
            self.plan_points = []
            self.distance_remaining = None
            self.nav_status = "Отправляю цель в Nav2…"

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback, s=seq: self._goal_feedback(feedback, s),
        )
        future.add_done_callback(lambda f, s=seq: self._goal_response(f, s))
        self.get_logger().info(
            f"Nav2 goal requested: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f} deg"
        )

    def _goal_response(self, future, seq):
        if seq != self.goal_seq:
            return
        try:
            handle = future.result()
        except Exception as exc:
            with self.lock:
                self.nav_status = f"Ошибка отправки цели: {exc}"
            return

        if not handle.accepted:
            with self.lock:
                self.nav_status = "Nav2 отклонил цель"
            return

        self.goal_handle = handle
        with self.lock:
            self.nav_status = "Еду к цели"
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f, s=seq: self._goal_result(f, s))

    def _goal_feedback(self, feedback_msg, seq):
        if seq != self.goal_seq:
            return
        try:
            distance = float(feedback_msg.feedback.distance_remaining)
        except Exception:
            return
        with self.lock:
            self.distance_remaining = distance
            self.nav_status = "Еду к цели"

    def _goal_result(self, future, seq):
        if seq != self.goal_seq:
            return
        try:
            result = future.result()
            status = result.status
        except Exception as exc:
            with self.lock:
                self.nav_status = f"Ошибка Nav2: {exc}"
                self.plan_points = []
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            text = "Цель достигнута"
        elif status == GoalStatus.STATUS_CANCELED:
            text = "Цель отменена"
        elif status == GoalStatus.STATUS_ABORTED:
            text = "Nav2 не смог достигнуть цели"
        else:
            text = f"Навигация завершена, status={status}"

        with self.lock:
            self.nav_status = text
            self.distance_remaining = None
            self.plan_points = []
        self.goal_handle = None
        self.get_logger().info(text)

    def _cancel_goal(self):
        handle = self.goal_handle
        if handle is None:
            with self.lock:
                self.nav_status = "Активной цели нет"
                self.plan_points = []
            return
        try:
            handle.cancel_goal_async()
            with self.lock:
                self.nav_status = "Отменяю цель…"
        except Exception as exc:
            with self.lock:
                self.nav_status = f"Ошибка отмены цели: {exc}"

    def destroy_node(self):
        try:
            self.httpd.shutdown()
            self.httpd.server_close()
        except Exception:
            pass
        try:
            self.nav_client.destroy()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationWebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
