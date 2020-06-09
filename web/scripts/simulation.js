var cw, ch;
var server_root = "http://localhost:8080/monitor";
var positions = null;

var og_robotWidth = 0.55;
var og_robotHeight = 0.47;

var robot_width;
var robot_height;


var stageScale = 20;
var arrow;
var backColor = '#f5f5f5';
var goalColor = '#3473ad';
var deliveryColor = '#24ad37';
var posRadius = 15;
var updateRate = 100; //ms
var simInterval;
var robotColors = null;
var colors = ['#404ff5', '#b109d6', '#db0f31', '#faff75', '#e0841b'];

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
        return;
    }

    renderTasks();
    renderRobots();
    renderAxis();
}

function renderAxis()
{
    ctx.setTransform(1,0,0,1,0,0);
    ctx.globalAlpha = 1;
    ctx.strokeStyle = '#000000';
    ctx.beginPath();
        ctx.moveTo(0, ch/2);
        ctx.lineTo(cw, ch/2);
    ctx.stroke();
 
    ctx.beginPath();
        ctx.moveTo(cw/2, 0);
        ctx.lineTo(cw/2, ch);
    ctx.stroke();

    fontSize = 14;
    ctx.font=fontSize+'px Courier New';
    ctx.fillStyle = '#000000';
    ctx.textAlign = 'center';
    y = (ch / 2) + (fontSize);
    ctx.fillText('0,0', cw/2, y);
    ctx.fillText('5', cw*3/4, y);
    ctx.fillText('10',cw-fontSize, y);
    ctx.fillText('-5',cw/4, y);
    ctx.fillText('-10',fontSize, y);

    x = (cw / 2) + (fontSize);
    ctx.fillText('5', x, ch/4);
    ctx.fillText('10',x,fontSize);
    ctx.fillText('-5',x,ch*3/4);
    ctx.fillText('-10',x,ch-fontSize);
    ctx.moveTo(0,0);
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


        ctx.translate(cx, cy);                      //T1
        ctx.rotate((-pos.yaw) * Math.PI / 180.0);   //T2
        ctx.translate(-cx, -cy);                    //T3
        // T1 * T2 * T3 * rect


        ctx.fillStyle = colors[i % colors.length];
        ctx.fillRect(x, y, robot_width, robot_height);
        ctx.fillStyle = '#000000';
        ctx.strokeRect(x, y, robot_width, robot_height);


        ctx.drawImage(arrow, x+7, cy-9, 30, 20);

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
    cw = $('#main_canvas').width();
    ch = $('#main_canvas').height();
    let scaleChange = cw / stageScale; 
    robot_width = og_robotWidth * scaleChange;
    robot_height = og_robotHeight * scaleChange;

    requestAnimationFrame(tick);
}