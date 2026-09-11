#!/usr/bin/env python3
import base64
import json
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
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
<title>turtle_gorod — карта</title>
<style>
  html,body{margin:0;width:100%;height:100%;overflow:hidden;background:#202124;color:#eee;font-family:Arial,sans-serif}
  #bar{position:absolute;z-index:2;left:12px;top:12px;background:rgba(20,20,20,.86);padding:10px 12px;border-radius:8px;font-size:14px;line-height:1.45;max-width:720px}
  #status{font-weight:700}
  #hint{color:#ffd166;margin-top:4px}
  canvas{display:block;width:100vw;height:100vh;cursor:crosshair}
</style>
</head>
<body>
<div id="bar">
  <div id="status">Ожидание карты…</div>
  <div id="info"></div>
  <div>чёрное — препятствия · красное — LiDAR · голубое — робот</div>
  <div id="hint">Для AMCL: зажмите ЛКМ на реальном положении робота и потяните в направлении его передней части.</div>
</div>
<canvas id="c"></canvas>
<script>
const canvas=document.getElementById('c');
const ctx=canvas.getContext('2d');
const statusEl=document.getElementById('status');
const infoEl=document.getElementById('info');
const hintEl=document.getElementById('hint');
let map=null, pose=null, scan=[];
let off=document.createElement('canvas'), offctx=off.getContext('2d');
let currentRev=-1;
let view=null;
let dragStart=null, dragNow=null;
let initialMarker=null;

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
  const img=new ImageData(arr,w,h);
  offctx.putImageData(img,0,0);
}

function worldToGrid(x,y){
  const dx=x-map.ox, dy=y-map.oy, c=Math.cos(map.oyaw), s=Math.sin(map.oyaw);
  return [(c*dx+s*dy)/map.res,(-s*dx+c*dy)/map.res];
}

function gridToWorld(gx,gy){
  const c=Math.cos(map.oyaw), s=Math.sin(map.oyaw);
  return [
    map.ox + (c*gx-s*gy)*map.res,
    map.oy + (s*gx+c*gy)*map.res,
  ];
}

function canvasToWorld(cx,cy){
  if(!map || !view)return null;
  const gx=(cx-view.ox)/view.scale;
  const gy=(view.oy+view.dh-cy)/view.scale;
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
  if(!map){ view=null; return; }

  const margin=28, scale=Math.min((W-2*margin)/map.w,(H-2*margin)/map.h);
  const dw=map.w*scale, dh=map.h*scale, ox=(W-dw)/2, oy=(H-dh)/2;
  view={scale, dw, dh, ox, oy};

  ctx.imageSmoothingEnabled=false;
  ctx.save();
  ctx.translate(ox,oy+dh); ctx.scale(1,-1);
  ctx.drawImage(off,0,0,dw,dh);
  ctx.restore();

  ctx.fillStyle='#ff3b30';
  for(const p of scan){
    const q=worldToCanvas(p[0],p[1]);
    if(q)ctx.fillRect(q[0]-1.3,q[1]-1.3,2.6,2.6);
  }

  if(initialMarker)drawArrow(initialMarker.x,initialMarker.y,initialMarker.yaw,'#ffb000',15);
  if(pose)drawArrow(pose.x,pose.y,pose.yaw,'#20d5e8',13);

  if(dragStart && dragNow){
    ctx.strokeStyle='#ffb000';
    ctx.lineWidth=3;
    ctx.beginPath();
    ctx.moveTo(dragStart.cx,dragStart.cy);
    ctx.lineTo(dragNow.cx,dragNow.cy);
    ctx.stroke();
  }
}

async function sendInitialPose(x,y,yaw){
  try{
    const r=await fetch('/initialpose',{
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({x,y,yaw}),
    });
    if(!r.ok)throw new Error('HTTP '+r.status);
    initialMarker={x,y,yaw};
    statusEl.textContent='Начальная позиция отправлена в AMCL';
    hintEl.textContent='Подождите несколько секунд: красные точки LiDAR должны совместиться со стенами карты.';
    draw();
  }catch(e){
    statusEl.textContent='Не удалось отправить initial pose';
  }
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
  const dx=cx-dragStart.cx, dy=cy-dragStart.cy;
  const pixels=Math.hypot(dx,dy);
  const start=dragStart;
  dragStart=null; dragNow=null;
  if(pixels<8){
    statusEl.textContent='Укажите направление робота';
    hintEl.textContent='Зажмите ЛКМ на позиции робота и потяните стрелку вперёд по направлению корпуса.';
    draw();
    return;
  }
  const end=canvasToWorld(cx,cy);
  if(!end)return;
  const yaw=Math.atan2(end[1]-start.y,end[0]-start.x);
  sendInitialPose(start.x,start.y,yaw);
});

async function tick(){
  try{
    const r=await fetch('/state?map_rev='+currentRev,{cache:'no-store'});
    if(!r.ok)throw new Error('HTTP '+r.status);
    const s=await r.json();
    if(s.map){ map=s.map; currentRev=map.rev; decodeMap(map.data,map.w,map.h); }
    pose=s.pose; scan=s.scan||[];
    if(map && !pose && !statusEl.textContent.startsWith('Начальная'))statusEl.textContent='Карта получена · AMCL ждёт начальную позицию';
    else if(map && pose)statusEl.textContent='Локализация активна';
    else if(!map)statusEl.textContent='Ожидание /map…';
    if(map){
      const p=pose?` · робот x=${pose.x.toFixed(2)} y=${pose.y.toFixed(2)} yaw=${(pose.yaw*180/Math.PI).toFixed(1)}°`:'';
      infoEl.textContent=`${map.w}×${map.h} · ${map.res.toFixed(3)} м/ячейку${p}`;
    }
    draw();
  }catch(e){ statusEl.textContent='Нет связи с роботом'; }
  setTimeout(tick,200);
}
resize(); tick();
</script>
</body>
</html>"""


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class MapWebNode(Node):
    def __init__(self):
        super().__init__("map_web")
        self.declare_parameter("port", 8080)
        self.port = int(self.get_parameter("port").value)

        self.lock = threading.Lock()
        self.map_revision = 0
        self.map_payload = None
        self.pose = None
        self.scan_points = []
        self.pending_initial_pose = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_timer(0.10, self._update_pose)
        self.create_timer(0.05, self._publish_pending_initial_pose)

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
                            "map": node.map_payload if client_rev != node.map_revision else None,
                        }
                    self._send_json(200, payload)
                    return

                self.send_error(404)

            def do_POST(self):
                parsed = urlparse(self.path)
                if parsed.path != "/initialpose":
                    self.send_error(404)
                    return

                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    data = json.loads(self.rfile.read(length).decode("utf-8"))
                    x = float(data["x"])
                    y = float(data["y"])
                    yaw = float(data["yaw"])
                    if not all(math.isfinite(v) for v in (x, y, yaw)):
                        raise ValueError("non-finite pose")
                    with node.lock:
                        node.pending_initial_pose = (x, y, yaw)
                    self._send_json(200, {"ok": True})
                except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
                    self._send_json(400, {"ok": False, "error": str(exc)})

        self.httpd = ThreadingHTTPServer(("0.0.0.0", self.port), Handler)
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(
            f"Web map: http://0.0.0.0:{self.port} "
            "(open this Raspberry Pi address in a browser)"
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

    def _publish_pending_initial_pose(self):
        with self.lock:
            pending = self.pending_initial_pose
            self.pending_initial_pose = None

        if pending is None:
            return

        x, y, yaw = pending
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)

        # Moderate uncertainty for a hand-selected initial pose.
        msg.pose.covariance[0] = 0.25 * 0.25
        msg.pose.covariance[7] = 0.25 * 0.25
        msg.pose.covariance[35] = math.radians(15.0) ** 2

        self.initial_pose_pub.publish(msg)
        self.get_logger().info(
            f"Initial pose sent: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f} deg"
        )

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
            r = msg.ranges[i]
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            mx = t.x + cy * lx - sy * ly
            my = t.y + sy * lx + cy * ly
            points.append([round(mx, 3), round(my, 3)])

        with self.lock:
            self.scan_points = points

    def destroy_node(self):
        try:
            self.httpd.shutdown()
            self.httpd.server_close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MapWebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
