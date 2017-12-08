/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "blackboard/blackboard.h"
#include <behavior_tree.h>
#include <dot_bt.h>
#include "behavior_tree_msgs/KeyValue.h"
#include "behavior_tree_msgs/UpdateBlackboard.h"

#include <ros/ros.h>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <algorithm>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

bool update_blackboard(behavior_tree_msgs::UpdateBlackboard::Request &req,
                       behavior_tree_msgs::UpdateBlackboard::Response &res,
                       boost::shared_ptr<BT::Blackboard> blkbrd_ptr)
{
    for (auto kv : req.kv_pairs)
    {
        switch (kv.value_type)
        {
            case behavior_tree_msgs::KeyValue::BOOL:
                // Convert expected 'boolean' string to lower case
                std::transform(kv.value.begin(), kv.value.end(), kv.value.begin(), ::tolower);

                if (kv.value == std::string("true"))
                    blkbrd_ptr->UpdateKv< bool >(kv.key, true);
                else if (kv.value == std::string("false"))
                    blkbrd_ptr->UpdateKv< bool >(kv.key, false);
                else
                {
                    res.keys_failed.push_back(kv.key);
                    ROS_ERROR_STREAM("[BEHAVIOR_TREE_CORE] Invalid Boolean representation for KeyValue Pair ("
                        << kv.key << ", " << kv.value << "). Ignoring");
                }
                break;

            case behavior_tree_msgs::KeyValue::INT:
                blkbrd_ptr->UpdateKv< int >(kv.key, atoi(kv.value.c_str()));
                break;

            case behavior_tree_msgs::KeyValue::DOUBLE:
                blkbrd_ptr->UpdateKv< double >(kv.key, atof(kv.value.c_str()));
                break;

            case behavior_tree_msgs::KeyValue::STRING:
                blkbrd_ptr->UpdateKv< std::string >(kv.key, kv.value);
                break;

            default:
                res.keys_failed.push_back(kv.key);
                ROS_ERROR_STREAM("[BEHAVIOR_TREE_CORE] Unsupported Type for KeyValue Pair (key: "
                    << kv.key << "). Ignoring");
                break;
        }
    }

    if (res.keys_failed.size() == 0)
        res.updated = behavior_tree_msgs::UpdateBlackboard::Response::SUCCESS;
    else
        res.updated = behavior_tree_msgs::UpdateBlackboard::Response::INVALID_KVS;

    return true;
}

void Execute(BT::ControlNode* root,
             int TickPeriod_milliseconds,
             boost::function<void()> tick_callback)
{
    BT::DotBt dotbt(root);
    std::thread t(&BT::DotBt::publish, dotbt);

    root->ResetColorState();

    while (ros::ok())
    {
        DEBUG_STDOUT("Ticking the root node !");

        // Ticking the root node
        root->Tick();

        if (tick_callback)
            tick_callback();

        if (root->get_status() != BT::RUNNING)
        {
            // when the root returns a status it resets the colors of the tree
            root->ResetColorState();
        }

        ResetFinishedNodes(root);

        // To process callbacks to any Action Clients and/or Services
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(TickPeriod_milliseconds));
    }
}


void Execute(BT::ControlNode* root,
             int TickPeriod_milliseconds,
             ros::NodeHandle& nh,
             boost::shared_ptr<BT::Blackboard> blkbrd_ptr,
             boost::function<void()> tick_callback)
{
    ros::ServiceServer service =
        nh.advertiseService<behavior_tree_msgs::UpdateBlackboard::Request,
                            behavior_tree_msgs::UpdateBlackboard::Response>
        ("set_blackboard_kvps", boost::bind(update_blackboard, _1, _2, blkbrd_ptr));

    Execute(root, TickPeriod_milliseconds, tick_callback);
}

void ResetFinishedNodes(BT::TreeNode* root)
{
    BT::ReturnStatus status = root->get_status();
    if (status == BT::FAILURE || status == BT::SUCCESS)
        root->set_status(BT::IDLE);

    if (root->get_type() == BT::NodeType::CONTROL_NODE)
    {
        BT::ControlNode* ctrl_node = dynamic_cast<BT::ControlNode*>(root);
        for (auto child : ctrl_node->GetChildren())
            ResetFinishedNodes(child);
    }
}

void GetLeafNodeStates(BT::TreeNode* root, std::vector<std::pair<BT::TreeNode*, BT::ReturnStatus>>& states)
{
    if (root->get_type() == BT::NodeType::CONTROL_NODE)
    {
        BT::ControlNode* ctrl_node = dynamic_cast<BT::ControlNode*>(root);
        for (auto child : ctrl_node->GetChildren())
            GetLeafNodeStates(child, states);
    }
    else
    {
        std::pair<BT::TreeNode*, BT::ReturnStatus> state(root, root->get_status());
        states.push_back(state);
    }
}
