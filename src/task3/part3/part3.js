import Simulation1 from "./simulation1.js";
// import Simulation2 from "./simulation2.js";
// import Simulation3 from "./simulation3.js";

class Timer {
  #timePoint = 0;

  reset() {
    this.#timePoint = performance.now();
  }

  getTime() {
    const currentTime = performance.now();
    return currentTime - this.#timePoint;
  }
}

const SimulationStates = {
  STOP: 0,
  RUN: 1,
  RESET: 2,
};

const sketch1 = (p5) => {
  const simulation = new Simulation1(p5);
  const timer = new Timer();
  let state = SimulationStates.RUN;
  let font;
  let canvas;

  p5.preload = () => {
    font = p5.loadFont("/assets/font/font.ttf");
  };

  p5.setup = () => {
    canvas = p5.createCanvas(400, 400, p5.WEBGL);
    canvas.parent("canvas1");

    p5.textFont(font);

    simulation.setCamera();
    timer.reset();
  };

  p5.draw = () => {
    const dt = timer.getTime();
    timer.reset();

    if (state == SimulationStates.RUN) {
      simulation.update(dt);
    }

    p5.background(200);
    simulation.draw();
  };
};
new p5(sketch1);

// const sketch2 = (p5) => {
//   const simulation = new Simulation2(p5);
//   const timer = new Timer();
//   let state = SimulationStates.RUN;
//   let font;

//   p5.preload = () => {
//     font = p5.loadFont("/assets/font/font.ttf");
//   };

//   p5.setup = () => {
//     p5.createCanvas(400, 400, p5.WEBGL);
//     p5.textFont(font);

//     timer.reset();
//   };

//   p5.draw = () => {
//     const dt = timer.getTime();
//     timer.reset();

//     if (state == SimulationStates.RUN) {
//       simulation.update(dt);
//     }

//     p5.background(200);
//     simulation.draw();
//   };
// };
// new p5(sketch2);

// const sketch3 = (p5) => {
//   const simulation = new Simulation3(p5);
//   const timer = new Timer();
//   let state = SimulationStates.RUN;
//   let font;

//   p5.preload = () => {
//     font = p5.loadFont("/assets/font/font.ttf");
//   };

//   p5.setup = () => {
//     p5.createCanvas(400, 400, p5.WEBGL);
//     p5.textFont(font);

//     timer.reset();
//   };

//   p5.draw = () => {
//     const dt = timer.getTime();
//     timer.reset();

//     if (state == SimulationStates.RUN) {
//       simulation.update(dt);
//     }

//     p5.background(200);
//     simulation.draw();
//   };
// };
// new p5(sketch3);
