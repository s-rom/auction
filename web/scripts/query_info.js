var robots_info = null;
var server_root = "http://localhost:8080/monitor";


$(document).ready(function(){
    $('#status_info').text('Waiting for monitor process to start...');
    setInterval(request_robots_info, 1000);
})

function request_robots_info() {
    $.ajax({url: server_root+"/get_robots_info",
        success: function (result) {
            $('#status_info').text(result);
            console.log("Response: "+result);
            $('#robots_table_body').empty();

            try{
                robots_info = JSON.parse(result);


                for (let i = 0; i < robots_info.robots.length; i++){
                    let id = robots_info.robots[i].id;
                    let host = robots_info.robots[i].host;
                    let port = robots_info.robots[i].port;


                    $('#robots_table> tbody:last-child').
                    append('<tr class=\'robot_row\'><td>'+id+'</td><td>'+host+'</td><td>'+port+'</td><td>ALIVE</td></tr>');
                }
            }
            catch(e){
                console.log("Output is not valid json");
            }
        }
    });
}

