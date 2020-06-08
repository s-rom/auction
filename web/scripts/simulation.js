var cw, ch;
var server_root = "http://localhost:8080/monitor";
var positions = null;
var robot_width = 12;
var robot_height = 18;
var stageScale = 20;
var arrow;
var backColor = '#f5f5f5';
var goalColor = '#3473ad';
var deliveryColor = '#24ad37';
var posRadius = 15;
var updateRate = 100; //ms
var simInterval;

function render(){
    ctx.globalAlpha = 1;
    ctx.fillStyle = backColor;
    ctx.fillRect(0,0,cw,ch);


    if (positions == null && tasks_info == null){
        ctx.strokeStyle = '#000000';
        ctx.fillStyle = '#000000';
        ctx.font='30px Courier New';
        ctx.fillText('System visualization', 70, ch/2);
        ctx.font='16px Courier New';
        ctx.fillText('Waiting for monitor server...', 120, ch/2+40);
        ctx.font = ''
    }

    renderTasks();
    renderRobots();
}

function stageToCanvasPoint(point){
    point[0] = point[0] * (cw/stageScale) + cw/2;
    point[1] = - point[1] * (ch/stageScale) + ch/2;   
    return point;
}

function parsePoint(pointStr){
    let len = pointStr.length;
    let delimIdx = pointStr.indexOf(",");
    let xStr = pointStr.substring(1, delimIdx);
    let yStr = pointStr.substring(delimIdx+1, len-1);

    let x = parseFloat(xStr);
    let y = parseFloat(yStr);

    return [x, y];
}

function renderTasks(){
    if (tasks_info == null) return;
    ctx.setTransform(1,0,0,1,0,0);

    ctx.globalAlpha = 0.4;

    for (let t = 0; t<tasks_info.tasks.length; t++){
        let task = tasks_info.tasks[t];

        if (task.status == "COMPLETED") continue;
        
        let goal = stageToCanvasPoint(parsePoint(task.goal));
        let delivery = stageToCanvasPoint(parsePoint(task.delivery));
        
        
        ctx.beginPath();
        ctx.arc(goal[0], goal[1], posRadius, 0, 2 * Math.PI);
        ctx.strokeStyle='#000000';
        ctx.stroke();
        ctx.fillStyle = goalColor;
        ctx.fill();

        ctx.beginPath();
        ctx.arc(delivery[0], delivery[1], posRadius, 0, 2 * Math.PI);
        ctx.strokeStyle='#000000';
        ctx.stroke();
        ctx.fillStyle = deliveryColor;
        ctx.fill();

        ctx.globalAlpha = 1;
        ctx.fillStyle='#000000';
        ctx.font = '12px consolas';
        ctx.strokeText(task.goal, goal[0], goal[1]);
        ctx.strokeText(task.delivery, delivery[0], delivery[1]);
    }
}

function renderRobots(){
    
    if (positions == null) return;
    ctx.globalAlpha = 1;

    for (let i = 0; i < positions.positions.length; i ++)
    {
        ctx.setTransform(1,0,0,1,0,0);
        let pos = positions.positions[i];
        
        let x = pos.x * (cw/stageScale) + cw/2;
        let y = -pos.y * (ch/stageScale) + ch/2;
        let cx = x + robot_width/2;
        let cy = y + robot_height/2;


        ctx.translate(cx, cy);
        ctx.rotate((-90-pos.yaw) * Math.PI / 180.0);
        ctx.translate(-cx, -cy);

        ctx.fillStyle = '#d4112b';
        ctx.fillRect(x, y, robot_width, robot_height);
        ctx.fillStyle = '#000000';
        ctx.strokeRect(x, y, robot_width, robot_height);
        ctx.fillStyle = '#3262a8';

        
        // ctx.beginPath();
        // ctx.arc(cx, cy, 20, 0, 2 * Math.PI);
        // ctx.stroke();

        ctx.translate(cx, cy);
        ctx.rotate(Math.PI/2);
        ctx.translate(-cx, -cy);
        ctx.drawImage(arrow, x, y, 30, 20);

    }


}

function queryPositions(){
    $.ajax({url: server_root+"/get_robots_positions",
        success: function (result) {
            try{
                positions = JSON.parse(result);
            } catch(e){
                console.log("Invalid JSON: "+result);
            }
        },
        error: function(){
            clearInterval(simInterval);
        }
    });
}


function update(deltaTime){
}

$(document).ready(function() {
    arrow = document.getElementById('source');
    simInterval = setInterval(queryPositions, updateRate);
    startSimulation();
});


var requestAnimationFrame = window.requestAnimationFrame;

var lastUpdate = new Date().getTime();

function tick() {
    cw = $('#main_canvas').width();
    ch = $('#main_canvas').height();
    var currentTime = new Date().getTime();
    var deltaTime = currentTime - lastUpdate;
    update(currentTime - lastUpdate);
    click = null;
    render();
    lastUpdate = currentTime;
    requestAnimationFrame(tick);
}


function startSimulation() {
    canvas = document.getElementById('main_canvas');
    ctx = canvas.getContext('2d');
    // todo: initialize state
    requestAnimationFrame(tick);
}