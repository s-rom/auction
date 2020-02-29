#include "RobotManager.h"

using Auction::RobotManager;


std::string get_log_path(std::string program_path)
{
    std::string path(program_path.substr(0,program_path.find("devel")));
    path += "src/auction/logs/";
    return path;
}

RobotManager::RobotManager(string program_path, NetProfile & net_info)
:
    message_system(net_info),
    info_report(get_log_path(program_path)+"robot"+std::string(net_info.port)+".log",
        Mode::COUT | Mode::FILE | Mode::TIME)
{
    message_system.request_unique_id();
    task_leader = -1;
}


RobotManager::RobotManager()
{
    task_leader = -1;
}


void RobotManager::close_info_reporter()
{
    info_report.close();
}


void RobotManager::message_server(boost::atomic<bool> & running)
{
    int socket_descriptor = RcSocket::passiveSocket(message_system.net_info.port,"udp", 0);
    if (socket_descriptor < 0)
    {
        return;
    }

    while (running)
    {
        struct sockaddr sender;
        uint addrlen;
        char msg[256];
        memset(&msg, 0, 256);
        // Recibe la logrmacion y la guarda en el buffer
        if (!recvfrom(socket_descriptor, msg, sizeof(msg), 0, &sender, &addrlen))
        {
            info_report << "[MessageServer] Error de recepcion \n";
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
    info_report << "[Auction process] ---> Running" << "\n";


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
                break;
            }
            case NEW_TASK:
            {
                NewTaskMessage * nt = dynamic_cast<NewTaskMessage*>(m);
                new_task_message_handler(*nt);
                break;
            }
            case LEADER_OF_TASK:
            {
                LeaderOfTaskMessage * lead_msg = dynamic_cast<LeaderOfTaskMessage*>(m);
                leader_of_task_handler(*lead_msg);
                break;
            }

            case BID_REQUEST:
            {
                SimpleMessage * bid_req = dynamic_cast<SimpleMessage*>(m);
                bid_request_message_handler(*bid_req);
                break;
            }

            case BID_FOR_TASK:
            {
                BidMessage * bid_msg = dynamic_cast<BidMessage*>(m);
                bid_for_task_message_handler(*bid_msg);
                break;
            }


            delete m;
        }
    }
}

void RobotManager::new_robot_message_handler(NewRobotMessage & nr)
{
    // net profile equals own net profile -> store unique id
    if (nr.np == message_system.net_info)
    {
        if (this->id == -1)
            info_report << "[NewRobotHandler] Received my own id: "<<nr.unique_id<<"\n";
        this->id = nr.unique_id;
    } 
    else // stores other robot id 
    {
        info_report << "[NewRobotHandler] Received other robot profile: "<<nr.np.to_string()<<
        ", "<<nr.unique_id<<"\n";
        message_system.add_robot_info(nr.unique_id, nr.np);
    }
}                

void RobotManager::new_task_message_handler(NewTaskMessage & nt)
{
    Task & new_task = nt.t;
    this->task_list[new_task.task_id] = new_task;
    info_report << "[NeTaskHandler] new task: "<<new_task.task_id << "\n";

    // If not a task leader, start a leader request process
    if (this->task_leader < 0)
        leader_request(new_task);

    // If leader of task
    if (this->task_leader >= 0)
    {
        // Start auction for this task
        Task & t = task_list[this->task_leader];
        this->leader_task_auction(t);
    }
}             

void RobotManager::leader_of_task_handler(LeaderOfTaskMessage & lead_msg)
{
    info_report << "[LeaderOfTaskHandler] Robot "<<lead_msg.robot_leader<<" is the new leader of task "
    <<lead_msg.task_id<<"\n";
}


void RobotManager::bid_request_message_handler(SimpleMessage &bid_req)
{
    // Generate bid for this task
    Task & t = task_list[bid_req.task_id];
    float bid = get_work_capacity(t);

    info_report << "[BidRequestMessageHandler]: Received a bid request from Robot" << 
        bid_req.robot_src << " for task " << bid_req.task_id << ".Replying with bid:" <<
        bid << "\n";

    // Send message with that bid    
    BidMessage my_bid(bid_req.task_id, this->id, bid, MessageType::BID_FOR_TASK);
    message_system.send_message(my_bid, bid_req.robot_src);
}

void RobotManager::bid_for_task_message_handler(BidMessage & bid_msg)
{
    // If not the leader for any task
    if (this->task_leader < 0)
    {
        Task & t = task_list[bid_msg.task_id];
        // Start the non-leader robot's auction algorithm (Algorithm 3)
        this->non_leader_task_auction(t, bid_msg);
    }

    // else do nothing
}



void RobotManager::leader_request(Task & t)
{
    info_report << "[LeaderAlgorithm] Starting a leader request process for task "<< t.task_id << "\n";
    
    // Initializes timer to 0
    auto time_init = system_clock::now();
    auto delta_time = duration_cast<std::chrono::milliseconds>
                        (time_init - time_init).count();
    
    // Bid's with work_capacity for that task
    float my_bid = this->get_work_capacity(t);

    // Broadcast's a message requesting this task
    BidMessage my_req(t.task_id, this->id, my_bid, MessageType::LEADER_REQUEST);
    info_report << "[LeaderAlgorithm] Robot "<< this->id <<" requesting task "
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
                    info_report << "[LeaderAlgorithm] " << lead_msg->robot_leader 
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
                        info_report << "[LeaderAlgorithm] Received bid " << other_bid
                            <<", and mine is "<<my_bid<<", giving up... \n";
                        return;
                    } 
                    // Other robot bid is less or equals to this robot's bid
                    else 
                    {
                        // Continue task request process
                        info_report << "[LeaderAlgorithm] Received bid " << other_bid
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
    info_report << "[LeaderAlgorithm] I'm the new leader for task "<<t.task_id<<"\n";
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
    info_report << "[TaskAuction] Starting auction round for task "<<t.task_id <<"\n";

    // int - robot_src_id
    // float - bid
    std::vector<std::pair<int,float>> group_bid;
    
    // vector of selected robots IDs 
    std::vector<int> selected_group;


    // Initializes timer to 0
    auto time_init = system_clock::now();
    auto delta_time = duration_cast<std::chrono::milliseconds>
                        (time_init - time_init).count();

    // Broadcast a bid_request for this task
    SimpleMessage bid_request(t.task_id, this->id, MessageType::BID_REQUEST);
    message_system.broadcast_message(bid_request);

    // FIRST AUCTION ROUND: Receive bids for this task
    while (delta_time < TIME_AUCTION)
    {
        // Creates a snapshot of the message queue
        auto end = message_queue.end();
        auto it = message_queue.begin();
        while (it != end)
        {
            Message * m = *it;
            if (m->type == MessageType::BID_FOR_TASK)
            {
                BidMessage * bid_msg = dynamic_cast<BidMessage*>(m);

                // Check that the message is refering this task
                if (bid_msg->task_id == t.task_id)
                {
                    info_report << "[AuctionForTask - Round 1] Received bid from Robot " <<
                        bid_msg->robot_src_id << ", with a bid of " << bid_msg->bid << "\n";
                    
                    // Stores id and bid 
                    // std::make_pair takes rvalue references as parameters.
                    // Copy them into variables to avoid segmentation fault
                    int lead_id = bid_msg->robot_src_id;
                    float lead_bid = bid_msg->bid;
                    group_bid.push_back(std::make_pair(lead_id, lead_bid));            

                    // Remove message from queue
                    message_queue.erase(it);

                    // Release memory
                    delete bid_msg;
                }
            }
            it++;
        }
        
        // Update delta time
        delta_time = duration_cast<std::chrono::milliseconds>
            (system_clock::now() - time_init).count();    
    }

    info_report << "[AuctionForTask - Round 1] End of round 1\n";

    // Sort the set (descending) by bid (second element) 
    std::sort(group_bid.begin(), group_bid.end(), sort_descending_by_second);
    // Deadline to be acomplished, DLj
    float goal_deadline = t.dead_line;
    
    // Large enough number to be considered infinity
    const float PSEUDO_INFINITY = 100000.0f;
    // Total accumulated expected time in function of the choosen group, DLg,j
    float accumulated_deadline = PSEUDO_INFINITY;

    float accum_group_work_capacity = 0.0f;

    for (auto it = group_bid.begin(); 
        it!=group_bid.end() && (accumulated_deadline >= goal_deadline); it++)
    {
        // Add first robot to the group
        int robot_id = it->first;
        float robot_bid = it->second;
        selected_group.push_back(robot_id);

        // Increments Sum of all group workcapacity 
        // with individual workcapacity (bid)
        accum_group_work_capacity += robot_bid;

        if (accum_group_work_capacity == 0)
        {
            // Avoids division by 0
            accumulated_deadline = PSEUDO_INFINITY;
        }
        else 
        {
            // Updates DLg,j
            accumulated_deadline = t.task_work_load / accum_group_work_capacity;
        }

        // TODO: Substract interference factor
    }

    // Compute expected task's utility
    float utility = t.utility_function(accumulated_deadline);

    // ---------------------------- DEBUG PURPOSES --------------------------------------
    info_report << "[AuctionForTask - Round 2] ---> Selected group: \n";
    for (auto it = selected_group.begin(); it != selected_group.end(); it++)
    {
        info_report << "\tRobot "<< *it << "\n";
    }
    info_report << "[AuctionForTask - Round 2] Total group_work_capacity: " << accum_group_work_capacity << "\n";
    info_report << "[AuctionForTask - Round 2] Expected deadline: " << accumulated_deadline << "\n";
    info_report << "[AuctionForTask - Round 2] TaskUtility(expected_deadline) <<Uj(DLgj)>>: "<<utility << "\n";
    //------------------------------------------------------------------------------------


    // SECOND AUCTION ROUND: Bid selected robots with Uj(DLg,j)
    for (auto it = selected_group.begin(); it != selected_group.end(); it++)
    {
        int robot_id = *it;
        BidMessage bid_msg(t.task_id, this->id, utility, MessageType::BID_FOR_TASK);
        message_system.send_message(bid_msg, robot_id);
    }

}

bool third_greater(const std::tuple<int,int,float> &a, 
                const std::tuple<int,int,float> &b)
{
    return std::get<2>(a) < std::get<2>(b);
}



void RobotManager::non_leader_task_auction(Task & t, BidMessage first_bid)
{
    // int - leader_id
    // int - task_id
    // float - leader_bid (utility expected)
    std::vector<std::tuple<int,int,float>> leader_bids;

    // Adds the first AWARD message (to reproduce line 2 of algorithm 3)
    info_report << "[NonLeaderAuction] Received bid from "<<first_bid.robot_src_id <<
        " for task " << t.task_id << " with a bid of " << first_bid.bid << "\n";

    leader_bids.push_back(std::make_tuple(first_bid.robot_src_id, 
                                        first_bid.task_id, 
                                        first_bid.bid));

    // Initializes timer to 0
    auto time_init = system_clock::now();
    auto delta_time = duration_cast<std::chrono::milliseconds>
                        (time_init - time_init).count();

    while (delta_time < TIME_BID_ACCEPTED)
    {

        // Creates a snapshot of the message queue
        auto it = message_queue.begin();
        auto end = message_queue.end();
        while (it != end)
        {
            Message * m = *it;
            if (m->type == MessageType::BID_FOR_TASK)
            {
                // Store bid
                BidMessage * bid_msg = dynamic_cast<BidMessage*>(m);
                leader_bids.push_back(std::make_tuple(bid_msg->robot_src_id, bid_msg->task_id, bid_msg->bid));

                info_report << "[NonLeaderAuction] Received bid from "<<bid_msg->robot_src_id <<
                    " for task " << bid_msg->task_id << " with a bid of " << bid_msg->bid << "\n";


                // Delete message from memory
                message_queue.erase(it);

                // Release memory
                delete bid_msg;
            }   

            it++;
        }

        // Update delta time
        delta_time = duration_cast<std::chrono::milliseconds>
            (system_clock::now() - time_init).count();
    } 

    auto max = *std::max_element(leader_bids.begin(),leader_bids.end(), third_greater);
    int best_leader = std::get<0>(max);
    int best_task = std::get<1>(max);
    float best_bid = std::get<2>(max);

    info_report << "[NonLeaderAuction] Best option: task "<<best_task << " from leader "<< best_leader
        << " whose bid is " << best_bid << "\n";

    SimpleMessage accept_msg(best_task, this->id,MessageType::ROBOT_ALIVE);
    message_system.send_message(accept_msg, best_leader);


    for (auto it = leader_bids.begin(); it != leader_bids.end(); it++)
    {
        int leader_refuse = std::get<0>(*it);
        int task_refuse = std::get<1>(*it);
        
        info_report << "[NonLeaderAuction] Sending refuse to\n";

        if (leader_refuse != best_leader)
        {
            info_report << "\t*Leader " << leader_refuse << "\n";

            SimpleMessage refuse_msg(task_refuse,this->id,MessageType::REFUSE);
            message_system.send_message(refuse_msg,leader_refuse);
        }
    }
}


void RobotManager::set_load_capacity(float new_load_capacity)
{
    this->load_capacity = (new_load_capacity < 0) ? 0 : new_load_capacity;
}


void RobotManager::set_max_linear_vel(float new_max_vel)
{
    this->max_vel = (new_max_vel < 0) ? 10 : new_max_vel;
}


float RobotManager::get_work_capacity(Task& t)
{
    float distance = Point2D::euclidean_distance(t.delivery_point,t.task_location);
    float a = load_capacity * max_vel;
    float b = 2 * (load_capacity * max_vel + distance);
    return b == 0? 0 : a / b;
}
