var cw, ch;
var server_root = "http://localhost:8080/monitor";
var positions = null;
var robot_width = 12;
var robot_height = 18;
var arrow;
var backColor = '#f5f5f5';
var updateRate = 100; //ms

function render(){
    ctx.fillStyle = backColor;
    ctx.fillRect(0,0,cw,ch);

    if (positions == null) return;

    for (let i = 0; i < positions.positions.length; i ++)
    {
        ctx.setTransform(1,0,0,1,0,0);
        let pos = positions.positions[i];
        
        let x = pos.x * (cw/20) + cw/2;
        let y = -pos.y * (ch/20) + ch/2;
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


        
        ctx.beginPath();
        ctx.arc(cx, cy, 20, 0, 2 * Math.PI);
        ctx.stroke();

        ctx.translate(cx, cy);
        ctx.rotate(Math.PI/2);
        ctx.translate(-cx, -cy);
        ctx.drawImage(arrow, x, y, 30, 20);

    }


    /* RENDER TASK LOCATIONS*/
    // if (tasks_info == null) return;
    // for (let t = 0; t<tasks_info.tasks.length; t++){
    //     let task = tasks_info.tasks[t];

    // }

}

function queryPositions(){
    $.ajax({url: server_root+"/get_robots_positions",
        success: function (result) {
            try{
                positions = JSON.parse(result);
            } catch(e){
                console.log("Invalid JSON: "+result);
            }
        }
    });
}


function update(deltaTime){
}

$(document).ready(function() {
    arrow = document.getElementById('source');
    setInterval(queryPositions, updateRate);
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