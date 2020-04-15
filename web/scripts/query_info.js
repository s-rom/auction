var robots_info = null;
var server_root = "http://localhost:8080/monitor";

$(document).ready(function(){
    $('#status_info').text('Waiting for monitor process to start...');
    setInterval(request_robots_info, 1000);
})

function add_new_task() {
    workload = $('#workload').val();
    deadline = $('#deadline').val() * 1000;
    x_goal = $('#goal_x').val();
    y_goal = $('#goal_y').val();
    x_delivery = $('#delivery_x').val();
    y_delivery = $('#delivery_y').val();
    mode = $("#new_task_form input[type='radio']:checked").val();


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
            $('#status_info').text(result);
            //console.log("Response: "+result);
            $('#robots_table_body').empty();

            try{
                robots_info = JSON.parse(result);


                for (let i = 0; i < robots_info.robots.length; i++){
                    let id = robots_info.robots[i].id;
                    let host = robots_info.robots[i].host;
                    let port = robots_info.robots[i].port;
                    let net_status = robots_info.robots[i].net_status;

                    let row_class;
                    if (net_status == "DEAD") row_class = "table-danger";
                    else row_class = "table-default";

                    // $('#robots_table> tbody:last-child').
                    // append('<tr class=\'robot_row\'><td>'+id+'</td><td>'+host+'</td><td>'+port+'</td><td>'+net_status+'</td>'+
                    // '<td>UNKNOWN</td>'+'</tr>');

                    let button;
                    if (net_status != "DEAD") 
                        button='<td><button type="button" class="btn btn-danger"'+
                            'onclick="on_click_kill('+id+');"'+'>Kill</button>';
                    else
                        button='<td><button type="button" class="btn btn-primary">Info</button>'; 
                    
                    $('#robots_table> tbody:last-child').
                    append('<tr class=\''+row_class+'\'><td>'
                            +id+'</td><td>'
                            +host+'</td><td>'
                            +port+'</td><td>'
                            +net_status+'</td>'
                            +'<td>UNKNOWN</td>'
                            +button+'</td>'
                            +'</tr>');
                    
                    
                }
            }
            catch(e){
                //console.log("Output is not valid json");
            }
        }
    });
}

