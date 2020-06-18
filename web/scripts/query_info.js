var robots_info = null;
var tasks_info = null;
var server_root = "http://localhost:8080/monitor";
var queryInterval;

$(document).ready(function(){
    $('#status_info').text('Waiting for monitor process to start...');
    queryInterval = setInterval(request_info, 1000);
})


function request_info()
{
    request_robots_info();
    request_tasks_info();
    $('#status_info').hide();
}

function add_new_task() {
    workload = $('#workload').val();
    deadline = $('#deadline').val() * 1000;
    x_goal = $('#goal_x').val();
    y_goal = $('#goal_y').val();
    x_delivery = $('#delivery_x').val();
    y_delivery = $('#delivery_y').val();
    mode = $("#new_task_form input[type='radio']:checked").val();


    if (workload == "" || 
        deadline == "" || 
        x_goal == "" || 
        y_goal == "" || 
        x_delivery == "" || 
        y_delivery == "" || 
        mode == "") 
    {
        alert("Please, fill all the fields for adding a new task!");
        return;
    }

    message = "_0_-1_"+x_goal+"_"+y_goal+"_"+x_delivery+"_"+y_delivery+"_"+workload+"_"+deadline+"_"+mode+"_";
    $.ajax({url: server_root+"/new_task/"+message, error: function(){}});
    
}

function on_click_kill(id) {
    console.log("Send kill robot to robot "+id);
    $.ajax({url: server_root+"/robot_kill/"+id, error: function(){}});
}

function request_robots_info() {
    $.ajax({url: server_root+"/get_robots_info",
        success: function (result) {
            //console.log("Response: "+result);
            $('#robots_table_body').empty();

            try{
                robots_info = JSON.parse(result);

                for (let i = 0; i < robots_info.robots.length; i++){
                    let id = robots_info.robots[i].id;
                    let host = robots_info.robots[i].host;
                    let port = robots_info.robots[i].port;
                    let net_status = robots_info.robots[i].net_status;
                    let role = robots_info.robots[i].role;

                    let row_class;
                    if (net_status == "DEAD") 
                        row_class = "table-danger";
                    else 
                        row_class = "table-default";

                    let button;
                    if (net_status != "DEAD") 
                        button='<td><button type="button" class="btn btn-danger"'+
                            'onclick="on_click_kill('+id+');"'+'>Kill</button>';
                    else
                        button='<td><button type="button" class="btn btn-danger" disabled >Kill</button>';

                    $('#robots_table> tbody:last-child').
                    append('<tr class=\''+row_class+'\'><td>'
                            +id+'</td><td>'
                            +host+'</td><td>'
                            +port+'</td><td>'
                            +net_status+'</td><td>'
                            +role+'</td>'
                            +button+'</td>'
                            +'</tr>');
                    
                    
                }
            }
            catch(e){
                //console.log("Output is not valid json");
            }
        }, error: function (){
            clearInterval(queryInterval); 
            alert("Please, refresh the page when the monitor is running.");
        }
    });
}

function request_tasks_info() {
    $.ajax({url: server_root+"/get_tasks_info",
        error: function(result){
            clearInterval(queryInterval);
        },
        success: function (result) {
            $('#tasks_table_body').empty();
            try{
                tasks_info = JSON.parse(result);

                for (let i = 0; i < tasks_info.tasks.length; i++){
                    
                    let id = tasks_info.tasks[i].id;
                    let workload = tasks_info.tasks[i].workload;
                    let delivery = tasks_info.tasks[i].delivery;
                    let goal = tasks_info.tasks[i].goal;
                    let deadline = tasks_info.tasks[i].deadline;
                    let status = tasks_info.tasks[i].status;
                    
                    if (status == "CONDUCTING")
                        row_class = "table-primary";
                    else 
                        row_class = "table-default";

                    $('#task_table> tbody:last-child').
                    append('<tr class=\''+row_class+'\'><td>'
                            +id+'</td><td>'
                            +workload.toFixed(1)+'</td><td>'
                            +delivery+'</td><td>'
                            +goal+'</td><td>'
                            +(deadline / 1000.0).toFixed(1)+'</td><td>'
                            +status+'</td>'
                            +'</tr>');
                }
            }
            catch(e){
                //console.log("Output is not valid json");
            }
        }
    });
}

