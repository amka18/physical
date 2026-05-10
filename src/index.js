import GameEngine from "./game_engine.js";

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

const sketch = (p5) => {
  const gameEngine = new GameEngine(p5);
  const timer = new Timer();

  p5.setup = () => {
    p5.createCanvas(700, 700, p5.WEBGL);
    gameEngine.init();
    timer.reset();
  };

  p5.draw = () => {
    const dt = timer.getTime();
    timer.reset();

    gameEngine.update(dt);

    p5.background(150);
    gameEngine.draw();
  };
};

new p5(sketch);
