#include "RobotManager.h"

using Auction::RobotManager;


RobotManager::RobotManager(NetProfile & net_info)
:
    message_system(net_info)
{
    message_system.request_unique_id();
    srand(time(0));
    task_leader = -1;
}

void RobotManager::message_server(boost::atomic<bool> & running)
{
    int socket_descriptor = RcSocket::passiveSocket(message_system.net_info.port,"udp", 0);
    if (socket_descriptor < 0)
    {
        // log.out("Error in RcSocket:: passiveSocket");
        return;
    }

    // log.out("[Message Server] ---> listening on port " + (message_system.net_info.port));

    while (running)
    {
        struct sockaddr sender;
        uint addrlen;
        char msg[256];
        memset(&msg, 0, 256);
        // Recibe la logrmacion y la guarda en el buffer
        if (!recvfrom(socket_descriptor, msg, sizeof(msg), 0, &sender, &addrlen))
        {
            std::cout << "Error de recepcion \n";
        }
        else
        { 
            Message * m = this->message_system.create_message_from(msg);
            message_queue.push(m);
        }
    }
}

void RobotManager::auction_process(boost::atomic<bool> & running)
{
    std::cout << "[Auction process] ---> Running" <<"\n";
    
    while(running)
    {
        if (message_queue.isEmpty()) continue;
        Message * m;
        message_queue.pop(m);        
        switch(m->type)
        {
            case NEW_ROBOT:
            {
                NewRobotMessage * nr = dynamic_cast<NewRobotMessage*>(m);
                new_robot_message_handler(*nr);
                delete nr;
                break;
            }
            case NEW_TASK:
            {
                NewTaskMessage * nt = dynamic_cast<NewTaskMessage*>(m);
                new_task_message_handler(*nt);
                delete nt;
                break;
            }
            case LEADER_OF_TASK:
            {
                LeaderOfTaskMessage * lead_msg = dynamic_cast<LeaderOfTaskMessage*>(m);
                leader_of_task_handler(*lead_msg);
                delete lead_msg;
                break;
            }
        }
        

    }
}

void RobotManager::new_robot_message_handler(NewRobotMessage & nr)
{
    // net profile equals own net profile -> store unique id
    if (nr.np == message_system.net_info)
    {
        this->id = nr.unique_id;
        std::cout << "[NewRobotHandler] Received my own id: "<<id<<"\n";
    } 
    else // stores other robot id 
    {
        std::cout << "[NewRobotHandler] Received other robot profile: "<<nr.np.to_string()<<
        ", "<<nr.unique_id<<"\n";
        message_system.add_robot_info(nr.unique_id, nr.np);
    }
}                

void RobotManager::new_task_message_handler(NewTaskMessage & nt)
{
    Task & new_task = nt.t;
    this->task_list[new_task.task_id] = new_task;
    std::cout << "[NeTaskHandler] new task: "<<new_task.task_id << "\n";

    if (this->task_leader < 0)
        leader_request(new_task);
}             

void RobotManager::leader_of_task_handler(LeaderOfTaskMessage & lead_msg)
{
    std::cout << "[LeaderOfTaskHandler] Robot "<<lead_msg.robot_leader<<" is the new leader of task "
    <<lead_msg.task_id<<"\n";
}


void RobotManager::leader_request(Task & t)
{
    std::cout << "[LeaderAlgorithm] Starting a leader request process for task "<< t.task_id << "\n";
    
    // Initializes timer to 0
    auto time_init = system_clock::now();
    auto delta_time = duration_cast<std::chrono::milliseconds>
                        (time_init - time_init).count();
    
    // Creates a random bid for testing purposes
    float my_bid = (rand() % 100) / 100.0f;

    // Broadcast's a message requesting this task
    BidMessage my_req(t.task_id, this->id, my_bid, MessageType::LEADER_REQUEST);
    std::cout << "[LeaderAlgorithm] Robot "<< this->id <<" requesting task "
        << t.task_id << ", bid: " << my_bid << "\n";
    message_system.broadcast_message(my_req);

    // Iterate's the algorithm for a given amount of time (RobotManager member: TIME_LEADERSHIP)
    while (delta_time < TIME_LEADERSHIP)
    {
        // Creates a snapshot of the message queue
        auto end = message_queue.end();
        auto it = message_queue.begin();
        while (it != end)
        {   
            Message  * m = *it;

            if (m->type == MessageType::LEADER_OF_TASK)
            {
                // Dynamic cast to child type
                LeaderOfTaskMessage * lead_msg = dynamic_cast<LeaderOfTaskMessage*>(m); 
                
                // Check that the message is refering this task
                if (lead_msg->task_id == t.task_id)
                {        
                    std::cout << "[LeaderAlgorithm] " << lead_msg->robot_leader 
                    << " is the new leader for task" << lead_msg->task_id 
                    << " Giving up process... \n";

                    // Pop iterator pointed object from message_queue
                    message_queue.erase(it);

                    // Delete object memory
                    delete m;

                    // Give up process
                    return;
                }
            }

            if (m->type == MessageType::LEADER_REQUEST)
            {  
                // Dinamic cast to child type
                BidMessage * lreq = dynamic_cast<BidMessage*>(m);

                if (lreq->task_id == t.task_id)
                {
                    float other_bid = lreq->bid;

                    // Pop iterator pointed object from message_queue
                    message_queue.erase(it);
                    
                    // Delete object memory
                    delete m;

                    // Other robot bid is higher than this robot's bid
                    if (other_bid > my_bid)
                    {
                        // Give up task request process
                        std::cout << "[LeaderAlgorithm] Received bid " << other_bid
                            <<", and mine is "<<my_bid<<", giving up... \n";
                        return;
                    } 
                    // Other robot bid is less or equals to this robot's bid
                    else 
                    {
                        // Continue task request process
                        std::cout << "[LeaderAlgorithm] Received bid " << other_bid
                            << ", but mine is "<<my_bid<<", resuming... \n";
                    }
                }
            }

            // Advance message_queue iterator
            it++;
        }

        // Update delta time
        delta_time = duration_cast<std::chrono::milliseconds>
                        (system_clock::now() - time_init).count();
    }

    // New leader
    std::cout << "[LeaderAlgorithm] I'm the new leader for task"<<t.task_id<<"\n";
    LeaderOfTaskMessage lead_msg(t.task_id, this->id);
    message_system.broadcast_message(lead_msg);
    task_leader = t.task_id;
}


bool sort_descending_by_second(const std::pair<int,float> &a, const std::pair<int,float> &b)
{
    return a.second > b.second;
}



void RobotManager::leader_task_auction(Task & t)
{
    std::cout << "[TaskAuction] Starting auction round for task "<<t.task_id <<"\n";

    // int - robot_src_id
    // float - bid
    std::vector<std::pair<int,float>> group_bid;


    // Initializes timer to 0
    auto time_init = system_clock::now();
    auto delta_time = duration_cast<std::chrono::milliseconds>
                        (time_init - time_init).count();


    // FIRST AUCTION ROUND: Receive bids for this task
    while (delta_time < TIME_AUCTION)
    {
        // Creates a snapshot of the message queue
        auto end = message_queue.end();
        auto it = message_queue.begin();

        Message * m = *it;
        if (m->type == MessageType::BID_FOR_TASK)
        {
            BidMessage * bid_msg = dynamic_cast<BidMessage*>(m);

            // Check that the message is refering this task
            if (bid_msg->task_id == t.task_id)
            {
                std::cout << "[AuctionForTask - Round1] Received bid from Robot " <<
                    bid_msg->robot_src_id << "with a bid of " << bid_msg->bid << "\n";
                // Stores id and bid
                group_bid.push_back(std::make_pair(bid_msg->robot_src_id, bid_msg->bid));
            }
        }
    }

    // Sort the set (descending) by bid (second element) 
    std::sort(group_bid.begin(), group_bid.end(), sort_descending_by_second);
    // Deadline to be acomplished, DLj
    float goal_deadline = t.dead_line;
    // Total accumulated deadline in function of the choosen group, DLg,j
    float accumulated_deadline = 0.0f;

    for (auto it = group_bid.begin(); 
        it!=group_bid.end() && (accumulated_deadline < goal_deadline); it++)
    {
        // Add first robot to the group
        // Update DLg,j
    }

    // SECOND AUCTION ROUND: Bid selected robots with Uj(DLg,j)


}

void RobotManager::non_leader_task_auction(Task & t)
{
    // TODO: Implement
}


float RobotManager::get_work_capacity(Task& t)
{
    float distance = Point2D::euclidean_distance(t.delivery_point,t.task_location);
    float a = LOAD_CAPACITY * V_MAX;
    float b = 2 * (LOAD_CAPACITY * V_MAX + distance);
    return b == 0? 0 : a / b;
}
