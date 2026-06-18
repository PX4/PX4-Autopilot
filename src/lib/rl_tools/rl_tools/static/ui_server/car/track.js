import {drawCar} from "./car.js";


const defaultParameters = {
    pixelSizeReal: 0.02, // in meters
    gridWidth: 100,
    gridHeight: 100,
    maxSteeringAngle: Math.PI / 4,
    trackColor: 'black',
    backgroundColor: 'white',
    carParameters: {lf: 0.029, lr: 0.033}
}

export class Track{
    constructor(canvas, parameters){
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.ctx.imageSmoothingEnabled = false

        this.pixelSize = 10;
        this.ratio = 1
        this.pixelOverlap = 1

        this.parameters = {...defaultParameters}
        this.parameters.gridWidth = parameters["width"]
        this.parameters.gridHeight = parameters["height"]
        this.parameters.pixelSizeReal = parameters["track_scale"]

        this.pixelSizeReal = this.parameters.pixelSizeReal;
        this.pixelToMeter = 1/this.pixelSizeReal;
        this.gridWidth = this.parameters.gridWidth;
        this.gridHeight = this.parameters.gridHeight;
        this.maxSteeringAngle = this.parameters.maxSteeringAngle;
        this.trackColor = this.parameters.trackColor;
        this.backgroundColor = this.parameters.backgroundColor;
        this.carParameters = this.parameters.carParameters;

        // state
        this.track = parameters["track"]
        this.enable_drawing = true;
        this.drawing = false;
        this.state = null;
        this.action = null;

        // hooks
        window.addEventListener('load', ()=>this.resizeCanvas());
        window.addEventListener('resize', ()=>this.resizeCanvas());
        canvas.addEventListener('mousedown', (e)=>{
            this.startDrawing(e)
            e.preventDefault()
        });
        canvas.addEventListener('mouseup', (e)=>{
            this.stopDrawing()
            e.preventDefault()
        });
        canvas.addEventListener('mouseout', (e)=>{
            this.stopDrawing()
            e.preventDefault()
        });
        canvas.addEventListener('mousemove', (e)=>{
            this.draw(e)
            e.preventDefault()
        });
        canvas.addEventListener('touchstart', (e)=>{
            console.log('touchstart', e)
            const touch = e.changedTouches[0];
            this.startDrawing(touch)

            e.preventDefault()
        });
        canvas.addEventListener('touchmove', (e) => {
            const touch = e.changedTouches[0];
            this.draw(touch)
            e.preventDefault()
        });
        canvas.addEventListener('touchend', (e)=>{
            this.stopDrawing()
            e.preventDefault()
        });
        this.resizeCanvas()
        this.animate()
    }
    pointInCanvas(x, y) {
        const rect = this.canvas.getBoundingClientRect();
        return x >= rect.left && x <= rect.right && y >= rect.top && y <= rect.bottom;
    }
    reset(){
        this.enable_drawing = true;
        this.track= Array(this.gridHeight).fill().map(() => Array(this.gridWidth).fill(false));
    }
    disable_drawing(){
        this.enable_drawing = false;
    }

    drawPixel(gridX, gridY) {
        if (gridX >= 0 && gridX < this.gridWidth && gridY >= 0 && gridY < this.gridHeight){
            this.ctx.fillStyle = 'black';
            this.ctx.fillRect(gridX * this.pixelSize - this.pixelOverlap, gridY * this.pixelSize - this.pixelOverlap, this.pixelSize + 2 * this.pixelOverlap, this.pixelSize + 2 * this.pixelOverlap);
            this.track[gridY][gridX] = true;
        }
    }

    redrawTrack() {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        for(let y = 0; y < this.gridHeight; y++){
            for(let x = 0; x < this.gridWidth; x++){
                if(this.track[y][x]){
                    this.ctx.fillStyle = this.trackColor;
                    this.ctx.fillRect(x * this.pixelSize - this.pixelOverlap, y * this.pixelSize - this.pixelOverlap, this.pixelSize + 2 * this.pixelOverlap, this.pixelSize + 2 * this.pixelOverlap);
                }
            }
        }
    }


    startDrawing(e){
        if(this.enable_drawing){
            this.drawing = true;
            this.draw(e);
        }
        else{
            this.drawing = false;
        }
    }

    stopDrawing() {
        this.drawing = false;
    }

    draw(e) {
        if (!this.drawing) return;

        const rect = this.canvas.getBoundingClientRect();
        const scaleFactor = {
            x: this.canvas.width / rect.width,
            y: this.canvas.height / rect.height
        };

        const x = Math.floor(((e.clientX - rect.left) * scaleFactor.x / this.ratio)/this.pixelSize - 0.5);
        const y = Math.floor(((e.clientY - rect.top) * scaleFactor.y / this.ratio)/this.pixelSize - 0.5);

        const brushSize = 3;

        for (let i = -brushSize*2; i < brushSize*2; i++) {
            for (let j = -brushSize*2; j < brushSize*2; j++) {
                if((i*i + j*j) <= brushSize*brushSize){
                    this.drawPixel(x + i, y + j);
                }
            }
        }
    }

    render() {
        this.redrawTrack()
        if(this.state){
            for(let car_i=0; car_i<this.state.length; car_i++){
                const action = this.action && this.action.length > car_i && this.action[car_i] && this.action[car_i].length == 1 && this.action[car_i][0].length == 2 ? this.action[car_i][0] : [0, 0];
                drawCar(this.canvas, this.ctx, this.carParameters, this.state[car_i], action, this.ratio, this.pixelSize/(this.pixelSizeReal));
            }
        }
    }

    resizeCanvas() {
        const canvasWidth = this.canvas.parentElement.offsetWidth;
        const canvasHeight = this.canvas.parentElement.offsetHeight;
        this.ratio = window.devicePixelRatio || 1;
        this.canvas.width = canvasWidth * this.ratio;
        this.canvas.height = canvasHeight * this.ratio;
        this.pixelSize = this.canvas.width / this.ratio / this.gridWidth;
        this.pixelToMeter = 1/this.pixelSizeReal;
        this.ctx.scale(this.ratio, this.ratio);
        this.canvas.style.width = `${this.canvas.width / this.ratio}px`;
        this.canvas.style.height = `${this.canvas.height / this.ratio}px`;
        this.render()
    }
    setSteering(steering) {
        this.action.steering = steering * this.maxSteeringAngle;
    }

    animate() {
        requestAnimationFrame(()=>this.animate());
        this.render();
    }
}








