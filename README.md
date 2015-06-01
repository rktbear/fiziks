# fiziks.js

## Overview

A simple physical body simulation for Javascript. Currently only supports linear momentum.

Demo can be found at: http://bheath.devio.us/fizdemo1.html (only supports Chrome and Firefox).

## Install

Place in the path of yoru HTML page and include with:

```
<script type="text/javascript" src="fiziks.js"></script>
```

## Usage

A canvas element must be added to the main body with an id. This canvas
must have equal width and height:

```
<canvas id="myCanvas" width=500 height=500></canvas>
```

Initialization of fiziks.js must occur once the page has loaded:

```
<script type="text/javascript">
  window.onload = function() {
    if(!fiziks.init("myCanvas")) {
      console.log("err");
      return; 
    }

    setInterval(function() {
      fiziks.update(30);
    }, 30);
  }
</script>
```

## Coordinate Spaces

Two spaces exist: canvas and fiziks space.

Canvas space is a coordinate space on the `<canvas>` element and has the range `[0,width]` and `[0,height]`. The coordinate `(0,0)` is the top left-hand corner of the element.

Fiziks space is used by the simulation and has the range `[-500,500]` and `[-500,500]`. The coordinate `(0,0)` is at the center of the space.

## Add a physics object

Only spheres can be created that move around the world based on forces.

Static entities that don't move but can act as collision for spheres are lines and containers.

A container is a box, made up of four lines, with normals pointing inwards (i.e. spheres will 
collide form the inside, but not the outside).

```
var sphereRef = fiziks.add(new ColSphere(new Vec3D(), 10 /* radius */));
var lineRef = fiziks.add(new ColLine(new Vec3D(), new Vec3D(10,10)));
var containerRef = fiziks.add(new ColContainer(new Vec3D(), 350 /* container size */));
```

Spheres can have forces applied:

```
var sphereRef = fiziks.add(new ColSphere(new Vec3D(), 10 /* radius */));
sphereRef.force = new Vec3D(0,-0.0001);
```

## Rendering

All physics entities can be mapped to canvas objects in canvas space and each have a render() function.

```
var canvas = document.getElementById("myCanvas");
var context = canvas.getContext("2d");
context.fillStyle = "#FFFFFF";
context.fillRect(0,0,canvas.width,canvas.height);
fiziks.getCanvasPrims().forEach(function(p) {
  p.render(context, app.getDebug());
});
```