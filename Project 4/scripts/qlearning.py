#!/usr/bin/env python3
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Naman Shah", "Rushang Karia"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Naman Shah"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import sys
import problem
import json
import os
import random
import utils
from tqdm.auto import trange
import time
import traceback
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from parser import parse_args
from server import initialize_planning_server
from server import generate_maze
from utils import initialize_ros
from utils import cleanup_ros


class QLearning:

    def __init__(self, objtypes, objcount, seed, file_path, alpha, gamma,
        episodes, max_steps, epsilon_task, env, clean):
        
        self.objtypes = objtypes
        self.objcount = objcount
        self.seed = seed
        self.epsilon_task = epsilon_task
        self.env = env
        self.obj_json_file = utils.ROOT_DIR + "/objects.json"
        self.obj = json.load(open(self.obj_json_file))
        self.helper = problem.Helper()
        self.helper.reset_world()
        
        assert not os.path.exists(file_path) or not os.path.isdir(file_path)
        
        self.file_path = file_path
        if clean:
            self.file_handle = open(file_path, "w")
            self.write_file_header(file_path)
        else:
            self.file_handle = open(file_path, "a")

        self.alpha = alpha
        self.gamma = gamma
        self.max_steps = max_steps

        q_values = self.learn(episodes)

        with open(utils.ROOT_DIR + "/q_values.json", "w") as fout:
            json.dump(q_values, fout)
            
    def write_file_header(self, file_path):
       
        with open(file_path, "w") as f:
            f.write("Env;Object Types;Num of Objects;Seed;Gamma;Episode #;Alpha;Epsilon;Cumulative Reward;Total Steps;Goal Reached\n")

    def write_to_file(self, file_path, episode_num, alpha, epsilon,
        cumulative_reward, total_steps, is_goal_satisfied):

        with open(file_path, "a") as f:
            f.write("%s;%u;%u;%u;%.6f;%u;%.6f;%.6f;%.2f;%u;%s\n" % (
                self.env,
                self.objtypes,
                self.objcount,
                self.seed,
                self.gamma,
                episode_num,
                alpha,
                epsilon,
                cumulative_reward,
                total_steps,
                is_goal_satisfied))

    def get_q_value(self,alpha, gamma, reward, q_s_a, q_s_dash_a_dash):
        '''
        Use the Q-Learning update rule to calculate and return the q-value.

        return type: float
        '''

        '''
        YOUR CODE HERE
        '''
        td = reward + gamma * q_s_dash_a_dash - q_s_a
        return q_s_a + alpha * td

    def compute_cumulative_reward(self, current_cumulative_reward, gamma, step, reward):
        '''
        Calculate the running cumulative reward at every step using 
        current value of the cumulative reward,
        discount factor (gamma), 
        current step number (step), 
        the rewards for the current state (reward)

        return type: float
        '''


        '''
        YOUR CODE HERE
        '''

        return current_cumulative_reward + ((gamma ** step) * reward)

    def get_epsilon(self, current_epsilon, episode):
        '''
        Calculate the value for decaying epsilon/
        
        Input: 
        current_epsilon: current value for the epsilon.
        episode: episode number

        Output:

        new value for the epsilon

        return type: float 
        '''

        '''
        YOUR CODE HERE

        '''

        floor = 0.01
        decay = 0.99
        
        if episode == 0:
            return current_epsilon - floor
        
        return max(floor, current_epsilon * decay)
    
    def alpha(self, current_alpha, episode, step):
        return current_alpha

    def learn(self, episodes):
        q_values = {} # Use this dictionary to keep track of q values

        root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
        actions_config_file = open(root_path + "/action_config.json",'r')
        actions_config = json.load(actions_config_file)

        objects_file = open(root_path + "/objects.json",'r')
        objects = json.load(objects_file)

        pick_loc=[]
        place_loc=[]
        for object in objects['object'].keys():
            for e in objects['object'][object]['load_loc']:
                pick_loc.append(e)
        
        for goal in objects['goal'].keys():
            for e in objects['goal'][goal]['load_loc']:
                place_loc.append(e)
        
        epsilon = 1.0
        for i in trange(0, episodes, desc="Episode", unit="episode"):

            epsilon = self.get_epsilon(epsilon, i) # Complete get_epsilon()
            curr_state = self.helper.get_current_state()
            cumulative_reward = 0
            
            for step in range(self.max_steps):

                if self.helper.is_terminal_state(curr_state):
                    break
                    
                actions_list = self.helper.get_all_actions()
                curr_loc = [curr_state['robot']['x'],curr_state['robot']['y']]    
                possible_actions_list = actions_list
                                    
                '''
                YOUR CODE HERE
                '''

                state_str = json.dumps(curr_state, sort_keys=True)
                q_values[state_str] = {action: 0.0 for action in actions_list}

                if random.random() < epsilon:
                    action = random.choice(possible_actions_list)
                else:
                    action = max(q_values[state_str], key = q_values[state_str].get)

                flag = action.split()
                action_only, object_name, goal_name = (flag + [None, None, None])[:3]

                action_params = {}

                if object_name:
                    action_params["object_name"] = object_name

                if goal_name:
                    action_params["goal_name"] = goal_name

                try:
                    result = self.helper.execute_action(action_only, action_params)
                    if result is None:
                        break
                    
                    flag, next_state = result
                    reward = self.helper.get_reward(curr_state, action_only, next_state)
                    cumulative_reward = self.compute_cumulative_reward(cumulative_reward, self.gamma, step, reward)

                    next_state_str = json.dumps(next_state, sort_keys=True)
                    q_values[next_state_str] = {a: 0.0 for a in actions_list}

                    next_max = max(q_values[next_state_str].values(), default=0.0)
                    curr_q_val = q_values[state_str][action]
                    q_values[state_str][action] = self.get_q_value(self.alpha, self.gamma, reward, curr_q_val, next_max)

                    curr_state = next_state
                    step = step + 1

                except Exception as e:
                    print("Exception during learning:", traceback.format_exc())
                    break

            self.write_to_file(self.file_path, i, self.alpha, epsilon,
                cumulative_reward, step, 
                self.helper.is_terminal_state(curr_state))
            self.helper.reset_world()

        return q_values

def run_qlearning(objtypes, objcount, seed, file_name, alpha, 
                  gamma, episodes, max_steps, epsilon_task, env, clean):
    
    file_path = utils.ROOT_DIR + "/" + file_name
    
    rosprocess = initialize_ros()
    planserver_process = initialize_planning_server()
    
    # Generate the world.
    generate_maze(objtypes, objcount, seed, 1, env)
   

    QLearning(objtypes, objcount, seed, file_path, alpha, 
        gamma, episodes, max_steps, epsilon_task, env, clean)
    
    cleanup_ros(planserver_process.pid, rosprocess.pid)
    time.sleep(2)


def submit(args):
    task_name = "task2"
    fname = "qlearning.csv"
    for i, env in enumerate(['cafeWorld','bookWorld']):
        print("Submission: running {} for {}".format(task_name, env))
        run_qlearning(objtypes=1, objcount=1, seed=100,
                      file_name=fname, alpha=0.3, gamma=0.9,
                      episodes=500, max_steps=500, epsilon_task=2,
                      env=env, clean=not(i))
    
    df = pd.read_csv(r"/home/aairobots/catkin_ws/src/hw3/qlearning.csv", delimiter = ';')

    for env in np.unique(df["Env"]):
        sub_df = df[df["Env"] == env]
        plt.figure(figsize = (16, 8))
        plt.title("Episode vs Cumulative Reward: {}".format(env))
        plt.xlabel("Episodes")
        plt.ylabel("Cumulative Reward")
        plt.plot(sub_df["Episode #"].values.tolist(), sub_df["Cumulative Reward"].values.tolist())
        plt.savefig(r"/home/aairobots/catkin_ws/src/hw3/{}_plot.png".format(env))

if __name__ == "__main__":

    random.seed(111)

    args = parse_args()

    if args.submit:
        submit(args)
    else:
        task_name = "task2"
        fname = "qlearning.csv"
        run_qlearning(objtypes=args.objtypes, objcount=args.objcount, seed=args.seed,
                      file_name=args.file_name, alpha=args.alpha, gamma=args.gamma,
                      episodes=args.episodes, max_steps=args.max_steps, epsilon_task=2,
                      env=args.env, clean=args.clean)