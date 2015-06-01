
function Vec3D(x,y,z) {
  this.x = x == undefined ? 0 : x; 
  this.y = y == undefined ? 0 : y;
  this.z = z == undefined ? 0 : z;
  this.copy = function() { return new Vec3D(this.x, this.y, this.z); };
  this.add = function(v) { this.x += v.x; this.y += v.y; this.z += v.z; return this;};
  this.sub = function(v) { this.x -= v.x; this.y -= v.y; this.z -= v.z; return this;};
  this.scale = function(s) { this.x *= s; this.y *= s; this.z *= s; return this; };
  this.neg = function() { this.scale(-1); return this; };
  this.normalize = function() { this.scale(1 / this.length()); return this; };
  this.dot = function(v) { return (this.x*v.x) + (this.y*v.y) + (this.z*v.z); };
  this.length = function() { return Math.sqrt(this.dot(this)); };
  this.cross = function(v) { 
    var cx = this.y*v.z - this.z*v.y;
    var cy = this.z*v.x - this.x*v.z;
    var cz = this.x*v.y - this.y*v.x;
    this.x = cx; this.y = cy; this.z = cz;
    return this;
  };
  return this;  
}

function Plane(p0,p1) {
  var vec = p0.copy().sub(p1).normalize();
  this.pos = p0;
  this.nrm = vec.copy().cross(zAxis);
  this.distToPos = function(pos) {
    return this.nrm.dot(pos.copy().sub(this.pos)); 
  }
}

var xAxis = new Vec3D(1,0,0);
var yAxis = new Vec3D(0,1,0);
var zAxis = new Vec3D(0,0,1);

function RigidBody(pos) {
  this.mass = 1;
  this.force = new Vec3D();
  this.vel = new Vec3D();
  this.prevPos = pos;
  this.pos = pos;
  this.verlet = function(dtSec) {
    // Acceleration: a = f / m
    // Verlet integration: x' = 2*x - xp + a*dt^2
    // Velocity is represented by distance travelled between two frames (x-xp).
    // Recommended solution for physics engines as it doesn't involve incremementing
    // floating point values each frame (e.g. as in Euler integration).
    var accDt2 = this.force.copy()
                     .scale(1 / this.mass)
                     .scale(dtSec*dtSec);
    var newPos = this.pos.copy()
                     .scale(2)
                     .sub(this.prevPos)
                     .add(accDt2);
    this.prevPos = this.pos.copy();
    this.pos = newPos;
    this.vel = (this.pos - this.prevPos);
  }
  this.euler = function(dtSec) {
    // Acceleration: a = f / m
    // Euler integration: x'=x+v  v=v+a
    // 
    var acc = this.force.copy().scale(1 / this.mass);
    this.prevPos = this.pos.copy();
    this.vel.add(acc.scale(dtSec));
    this.pos.add(this.vel.copy().scale(dtSec));
  }
  // this.integrate = this.verlet;
  this.integrate = this.euler;
  return this;
}

function ColSphere(pos, radius, mass) {
  var that = new RigidBody(pos);
  that.mass = mass ? mass : 1;
  that.colType = 'sphere';
  that.radius = radius;
  return that;
}

function ColLine(pos0, pos1) {
  this.colType = 'line';
  this.pos0 = pos0;
  this.pos1 = pos1;
  this.updatePlane = function() {
    this.plane = new Plane(this.pos0, this.pos1);
  }
  this.updatePlane();
  return this;  
}

// Only to be used to contain other collision primitives (because
// the box normals face into the box).
function ColContainer(cen, size) {
  this.colType = 'container';
  this.cen = cen;
  this.size = size;
  this.lines = [];
  var points = [
    [new Vec3D(0,1), new Vec3D(-1,0)], 
    [new Vec3D(1,0), new Vec3D(0,1)],
    [new Vec3D(0,-1), new Vec3D(1,0)], 
    [new Vec3D(-1,0), new Vec3D(0,-1)], 
  ];
  var linesPtr = this.lines;
  points.forEach(function(p) {
    var p0 = p[0].copy().scale(size).add(cen);
    var p1 = p[1].copy().scale(size).add(cen);
    linesPtr.push(new ColLine(p0,p1));
  });
  return this;
}

var collisionMap = {
  'sphere-line': {
    collide: function(s,l) {
      // Distance of sphere position to plane: d = dot(N, P-P0)
      var dc = l.plane.distToPos(s.pos);
      var dp = l.plane.distToPos(s.prevPos);
      // Test: 
      //  - Have sphere centers crossed the line since last frame.
      //  - Does sphere radius intersect with line.
      if(Math.sign(dc) != Math.sign(dp) || Math.abs(dc) <= s.radius) {
        return {collide:true};
      }
      return {collide:false};
    },
    response: function(s,l) {
      // Push s current position away from the line: rp = -(d - r).N + P
      // Leads to sphere's sliding against lines at an angle.
      var d = l.plane.distToPos(s.pos);
      var offset = new Vec3D();
      if(d >= 0) {
        offset = l.plane.nrm.copy()
                  .scale(s.radius-d);
      } else {
        offset = l.plane.nrm.copy()
                  .scale(s.radius+Math.abs(d));
      }
      
      //TODO: Dampen velocity!!!!!
      s.pos.add(offset);
    }
  },

  'sphere-sphere': {
    collide: function(s0,s1) {
      // Distance between two spheres.
      var d = s1.pos.copy().sub(s0.pos).length();
      if(d < s0.radius+s1.radius) {
        return {collide:true};
      }
      return {collide:false};
    },
    response: function(s0,s1) {
      var vs = s1.pos.copy().sub(s0.pos);
      var d = vs.length();
      vs.normalize();
      var mv = vs.copy()
                 .scale(s0.radius+s1.radius-d)

      s0.pos.sub(mv.copy().scale(0.5));
      s1.pos.add(mv.copy().scale(0.5));
    }
  },

  'sphere-container': {
    collide: function(s,c) {
      var contactLines = [];
      for(var i = 0, ilen = c.lines.length; i < ilen; i++) {
        var col = collisionMap['sphere-line'].collide(s,c.lines[i]);
        if(col.collide) {
          contactLines.push(c.lines[i]);
        }
      }
      return {collide:contactLines.length>0, data:contactLines};
    },
    response: function(s,c,contactLines) {
      contactLines.forEach(function(line) {
        collisionMap['sphere-line'].resolve(s,line);
      });
    }
  }
};

function primCollide(p0,p1) { 
  var fnName = p0.colType+"-"+p1.colType;
  if(collisionMap[fnName]) {
    return collisionMap[fnName].collide(p0,p1);
  }
  return false;
}

function collisionResponse(p0,p1,contactData) {
  var fnName = p0.colType+"-"+p1.colType;
  collisionMap[fnName] && collisionMap[fnName].resolve(p0,p1,contactData);
}

function integrate(primList, timeDeltaMS) {
  primList.forEach(function(p) {
    p.integrate && p.integrate(1 / timeDeltaMS);
  });
}

function collision(primList) {
  primList.forEach(function(p0) {
    primList.forEach(function(p1) {
      if(p0 == p1) return;
      var contactData = primCollide(p0,p1);
      if(contactData.collide) {
        collisionResponse(p0,p1,contactData.data);
      }
    });
  });
}

var fiziks = (function() {
  var width = 0;
  var height = 0;
  var prims = [];

  function posFizToCanvas(x,y) {
    var cx = ((x + 500) / 1000) * width;
    var cy = ((-y + 500) / 1000) * height;
    return { x:cx, y:cy };
  }

  function posCanvasToFiz(x,y) {
    var fx = (1000 * x / width) - 500;
    var fy = -((1000 * y / height) - 500);    
    return { x:fx, y:fy };
  }

  function init(canvasName) {
    var canvas = document.getElementById(canvasName);
    if (canvas == null) {
      console.log("ERROR: Unable to find canvas '"+canvasName+"'.");
      return false;
    }
    if (canvas.width != canvas.height) {
      console.log("ERROR: Canvas must have equal width and height!");
      return false;
    }
    width = canvas.width;
    height = canvas.height;
    return true;
  }

  function update(timeDeltaMS) {
    integrate(prims, timeDeltaMS);
    for(var i = 0; i < 20; i++) {
      collision(prims);
    };
  }

  function CanvasCircle(p) {
    var canvasPos = posFizToCanvas(p.pos.x, p.pos.y)
    this.x = canvasPos.x;
    this.y = canvasPos.y;
    this.r = (p.radius / 1000) * width;
    this.vx = p.vel.x;
    this.vy = -p.vel.y;
    this.render = function(ctx,debug) {
      // Draw the sphere.
      ctx.strokeStyle = "#000000";
      ctx.beginPath();
      ctx.arc(this.x, this.y, this.r, 0, Math.PI*2);
      ctx.stroke();  
      // Draw a velocity line.
      if(debug) {
        ctx.strokeStyle = "#0000FF";
        ctx.beginPath();
        ctx.moveTo(this.x, this.y);
        ctx.lineTo(this.x+(this.vx*5), this.y+(this.vy*5));
        ctx.stroke();
      }
    }
  }

  function CanvasLine(p) {
    var canvasPos0 = posFizToCanvas(p.pos0.x, p.pos0.y);
    var canvasPos1 = posFizToCanvas(p.pos1.x, p.pos1.y);
    this.x0 = canvasPos0.x;
    this.y0 = canvasPos0.y;
    this.x1 = canvasPos1.x;
    this.y1 = canvasPos1.y;
    this.nx = p.plane.nrm.x;
    this.ny = -p.plane.nrm.y;
    this.render = function(ctx,debug) {
      // Draw the line.
      ctx.strokeStyle = "#000000";          
      ctx.beginPath();
      ctx.moveTo(this.x0, this.y0);
      ctx.lineTo(this.x1, this.y1);
      ctx.stroke(); 
      // Draw the line's normal at the line midpoint.
      if(debug) {
        ctx.strokeStyle = "#FF0000";
        var midX = this.x0 + ((this.x1 - this.x0) * 0.5)
        var midY = this.y0 + ((this.y1 - this.y0) * 0.5)
        ctx.beginPath();
        ctx.moveTo(midX, midY);
        ctx.lineTo(midX+(this.nx*20), midY+(this.ny*20));
        ctx.stroke();
      }
    }
  }

  function CanvasContainer(p) {
    this.lines = p.lines.map(function(l) {
      return new CanvasLine(l);
    });
    this.render = function(ctx,debug) {
      this.lines.forEach(function(l) {
        l.render(ctx,debug);
      });
    }
  }

  var primToCanvas = {
    'line': CanvasLine,
    'sphere': CanvasCircle,
    'container': CanvasContainer
  }

  return {
    init: init,
    update: update,
    add: function(p) { prims.push(p); return p; },
    clear: function() { prims = []; },
    count: function() { return prims.length; },
    mapFizToCanvas: posFizToCanvas, // Map a canvas position to fiziks space.
    mapCanvasToFiz: posCanvasToFiz, // Map a fiziks position to canvas space.
    getCanvasPrims: function() { return prims.map(function(p) {
      return new primToCanvas[p.colType](p);
    })},    
  };
})();