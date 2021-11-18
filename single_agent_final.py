import mapc2020
import random
import itertools
import operator


class Agent:
    def __init__(self, agent, map_size, current_node, extra_vision, not_explored_weight, obstacle_weigth):
        self.agent = agent
        self.map_size = map_size  # (40, 40)
        self.current_node = current_node  # (15, 10)
        self.extra_vision = extra_vision
        self.not_explored_weight = not_explored_weight
        self.obstacle_weigth = obstacle_weigth
        self.seeking = 0
        self.goals = set()
        self.obstacles = set()
        self.dispensers = set()
        self.task_boards = set()
        self.blocks = set()
        self.map = [[not_explored_weight] * map_size[1]
                    for _ in range(map_size[0])]
        self.exploration_map = [[0] * map_size[1] for _ in range(map_size[0])]
        self.terrains = agent.dynamic["terrain"]
        self.things = agent.dynamic["things"]
        self.tasks = self.agent.dynamic["tasks"]
        self.directions_char = ["n", "w", "s", "e"]
        self.goal_found = False
        self.goal_found_vision = False
        self.goal_coordinate_x = 0
        self.goal_coordinate_y = 0
        self.cnt = 0
        self.task_accepted = False
        self.requirement_pos = ()
        self.chosen_task_name = ""
        self.dis_pos = ""
        self.found_dispenser = False
        self.last_dir = "n"
        self.is_attached = False
        self.block_detail = ""

    def range_checker_vertical(self, y):
        if y >= self.map_size[1]:
            return y % self.map_size[1]
        elif y < 0:
            return self.map_size[1] + y
        return y

    def range_checker_horizontal(self, x):
        if x >= self.map_size[0]:
            return x % self.map_size[0]
        elif x < 0:
            return self.map_size[0] + x
        return x

    def _get_dynamics(self):
        self.terrains = self.agent.dynamic["terrain"]
        self.things = self.agent.dynamic["things"]
        self.tasks = self.agent.dynamic["tasks"]

    def is_reachable(self, target):
        map_tracker = [[0] * self.map_size[1] for _ in range(self.map_size[0])]
        map_tracker[target[0]][target[1]] = 1
        m = self.map
        queue = []
        depth = 0
        elem = (target[0], target[1], depth)
        queue.append(elem)
        reachable = False
        cnt = 0
        x1 = self.current_node[0]
        y1 = self.current_node[1]
        ans = ''

        if(m[target[0]][target[1]] != 0):
            return False, 0, ''

        if(target[0] == x1 and target[1] == y1):
            return True, 0, ''

        while((len(queue) > 0) and (not reachable) and (cnt < self.map_size[0] * self.map_size[1])):
            cnt += 1
            q = queue[0]
            # del queue[0]
            queue.pop(0)
            x = q[0]
            y = q[1]
            depth = q[2] + 1
            if(x == x1 and y == self.range_checker_vertical(y1 - 1)):
                reachable = True
                ans = 'n'
            elif(x == x1 and y == self.range_checker_vertical(y1 + 1)):
                reachable = True
                ans = 's'
            elif(x == self.range_checker_horizontal(x1 - 1) and y == y1):
                reachable = True
                ans = 'w'
            elif(x == self.range_checker_horizontal(x1 + 1) and y == y1):
                reachable = True
                ans = 'e'

            x_1 = self.range_checker_horizontal(x - 1)
            x_2 = self.range_checker_horizontal(x + 1)
            y_1 = self.range_checker_vertical(y - 1)
            y_2 = self.range_checker_vertical(y + 1)
            if(m[x_1][y] == 0 and map_tracker[x_1][y] == 0):
                queue.append((x_1, y, depth))
                map_tracker[x_1][y] = 1
            if(m[x_2][y] == 0 and map_tracker[x_2][y] == 0):
                queue.append((x_2, y, depth))
                map_tracker[x_2][y] = 1
            if(m[x][y_1] == 0 and map_tracker[x][y_1] == 0):
                queue.append((x, y_1, depth))
                map_tracker[x][y_1] = 1
            if(m[x][y_2] == 0 and map_tracker[x][y_2] == 0):
                queue.append((x, y_2, depth))
                map_tracker[x][y_2] = 1
        return reachable, depth, ans

    def is_reachable_with_box(self, target):
        map_tracker = [[0] * self.map_size[1] for _ in range(self.map_size[0])]
        directions = {"n": [0, -1], "w": [-1, 0], "s": [0, 1], "e": [1, 0]}
        dir = directions[self.dis_pos]
        map_tracker[target[0]][target[1]] = 1
        map_tracker[self.range_checker_horizontal(
            target[0] + dir[0])][self.range_checker_vertical(target[1] + dir[1])] = 1
        m = self.map
        queue = []
        depth = 0
        elem = (target[0], target[1], depth)
        queue.append(elem)
        reachable = False
        cnt = 0
        x1 = self.current_node[0]
        y1 = self.current_node[1]
        ans = ''

        if(m[target[0]][target[1]] != 0 or m[self.range_checker_horizontal(target[0] + dir[0])][self.range_checker_vertical(target[1] + dir[1])] != 0):
            return False, 0, ''

        # if self.is_attached:
        #     d
        # else:
        if(target[0] == x1 and target[1] == y1):
            return True, 0, ''

        while((len(queue) > 0) and (not reachable) and (cnt < self.map_size[0] * self.map_size[1])):
            cnt += 1
            q = queue[0]
            queue.pop(0)
            x = q[0]
            y = q[1]
            depth = q[2] + 1

            dis_pos_x = self.range_checker_horizontal(x + dir[0])
            dis_pos_y = self.range_checker_vertical(y + dir[1])
            dis_N = m[self.range_checker_horizontal(
                dis_pos_x + directions["n"][0])][self.range_checker_vertical(dis_pos_y + directions["n"][1])]
            dis_W = m[self.range_checker_horizontal(
                dis_pos_x + directions["w"][0])][self.range_checker_vertical(dis_pos_y + directions["w"][1])]
            dis_S = m[self.range_checker_horizontal(
                dis_pos_x + directions["s"][0])][self.range_checker_vertical(dis_pos_y + directions["s"][1])]
            dis_E = m[self.range_checker_horizontal(
                dis_pos_x + directions["e"][0])][self.range_checker_vertical(dis_pos_y + directions["e"][1])]

            if(x == x1 and y == self.range_checker_vertical(y1 - 1) and dis_N == 0):
                reachable = True
                ans = 'n'
            elif(x == x1 and y == self.range_checker_vertical(y1 + 1) and dis_S == 0):
                reachable = True
                ans = 's'
            elif(x == self.range_checker_horizontal(x1 - 1) and y == y1 and dis_W == 0):
                reachable = True
                ans = 'w'
            elif(x == self.range_checker_horizontal(x1 + 1) and y == y1 and dis_E == 0):
                reachable = True
                ans = 'e'

            x_1 = self.range_checker_horizontal(x - 1)
            x_2 = self.range_checker_horizontal(x + 1)
            y_1 = self.range_checker_vertical(y - 1)
            y_2 = self.range_checker_vertical(y + 1)
            if(m[x_1][y] == 0 and map_tracker[x_1][y] == 0 and dis_W == 0):
                queue.append((x_1, y, depth))
                map_tracker[x_1][y] = 1
            if(m[x_2][y] == 0 and map_tracker[x_2][y] == 0 and dis_E == 0):
                queue.append((x_2, y, depth))
                map_tracker[x_2][y] = 1
            if(m[x][y_1] == 0 and map_tracker[x][y_1] == 0 and dis_N == 0):
                queue.append((x, y_1, depth))
                map_tracker[x][y_1] = 1
            if(m[x][y_2] == 0 and map_tracker[x][y_2] == 0 and dis_S == 0):
                queue.append((x, y_2, depth))
                map_tracker[x][y_2] = 1
        return reachable, depth, ans

    def _explore(self, extra_vision):
        directions = {0: [0, -1], 1: [-1, 0], 2: [0, 1], 3: [1, 0]}
        # North
        sum_dir = 0
        cnt = 0
        for i in range(-4 + ((-1) * extra_vision), 5 + extra_vision):
            for j in range((5 + extra_vision - abs(i)) * (-1), 0):
                sum_dir += self.map[self.range_checker_horizontal(
                                    self.current_node[0] + i)][self.range_checker_vertical(self.current_node[1] + j)]

        x4 = self.range_checker_horizontal(
            self.current_node[0] + directions[cnt][0])
        y4 = self.range_checker_vertical(
            self.current_node[1] + directions[cnt][1])
        is_not_explored = self.exploration_map[x4][y4]
        if(self.last_dir == "s"):
            arr = {cnt: sum_dir - 1000}
        elif(is_not_explored == 1):
            arr = {cnt: sum_dir - 100}
        else:
            arr = {cnt: sum_dir}
        cnt += 1

        # West
        sum_dir = 0
        for i in range(-4 + ((-1) * extra_vision), 0):
            for j in range((5 + extra_vision - abs(i)) * (-1), 6 + extra_vision - abs(i)):
                sum_dir += self.map[self.range_checker_horizontal(
                                    self.current_node[0] + i)][self.range_checker_vertical(self.current_node[1] + j)]

        x4 = self.range_checker_horizontal(
            self.current_node[0] + directions[cnt][0])
        y4 = self.range_checker_vertical(
            self.current_node[1] + directions[cnt][1])
        is_not_explored = self.exploration_map[x4][y4]
        if(self.last_dir == "e"):
            dummy = {cnt: sum_dir - 1000}
        elif(is_not_explored == 1):
            dummy = {cnt: sum_dir - 100}
        else:
            dummy = {cnt: sum_dir}
        arr.update(dummy)
        cnt += 1

        # South
        sum_dir = 0
        for i in range(-4 + ((-1) * extra_vision), 5 + extra_vision):
            for j in range(1, 6 + extra_vision - abs(i)):
                sum_dir += self.map[self.range_checker_horizontal(
                                    self.current_node[0] + i)][self.range_checker_vertical(self.current_node[1] + j)]

        x4 = self.range_checker_horizontal(
            self.current_node[0] + directions[cnt][0])
        y4 = self.range_checker_vertical(
            self.current_node[1] + directions[cnt][1])
        is_not_explored = self.exploration_map[x4][y4]
        if(self.last_dir == "n"):
            dummy = {cnt: sum_dir - 1000}
        elif(is_not_explored == 1):
            dummy = {cnt: sum_dir - 100}
        else:
            dummy = {cnt: sum_dir}
        arr.update(dummy)
        cnt += 1

        # East
        sum_dir = 0
        for i in range(1, 6 + extra_vision):
            for j in range((5 + extra_vision - abs(i)) * (-1), 6 + extra_vision - abs(i)):
                sum_dir += self.map[self.range_checker_horizontal(
                                    self.current_node[0] + i)][self.range_checker_vertical(self.current_node[1] + j)]

        x4 = self.range_checker_horizontal(
            self.current_node[0] + directions[cnt][0])
        y4 = self.range_checker_vertical(
            self.current_node[1] + directions[cnt][1])
        is_not_explored = self.exploration_map[x4][y4]
        if(self.last_dir == "w"):
            dummy = {cnt: sum_dir - 1000}
        elif(is_not_explored == 1):
            dummy = {cnt: sum_dir - 100}
        else:
            dummy = {cnt: sum_dir}
        arr.update(dummy)
        cnt += 1

        d_descending = sorted(
            arr.items(), key=operator.itemgetter(1), reverse=True)
        return d_descending

    def solve_task(self):
        mn = 1600
        reachability = False
        name = ""
        answer = ""
        dis_pos = ()
        requirement_pos1 = ()
        directions = {0: [0, -1], 1: [-1, 0], 2: [0, 1], 3: [1, 0]}
        for dispenser in self.dispensers:
            for task in self.tasks:
                for p in range(4):
                    x2 = self.range_checker_horizontal(
                        directions[p][0] + dispenser[1][0])
                    y2 = self.range_checker_vertical(
                        directions[p][1] + dispenser[1][1])
                    reach, depth, answer1 = agent.is_reachable((x2, y2))
                    if(reach and task["requirements"][0]["type"] == dispenser[0] and depth < mn):
                        mn = depth
                        reachability = True
                        answer = answer1
                        requirement_pos1 = (x2, y2)
                        name = task["name"]
                        dis_pos = self.directions_char[p]
        return reachability, mn, answer, name, requirement_pos1, dis_pos

    def agent_move(self, answer):
        self.agent.move(answer)
        directions = {"n": [0, -1], "w": [-1, 0], "s": [0, 1], "e": [1, 0]}
        dir = directions[answer]
        self.current_node = (self.range_checker_horizontal(
            self.current_node[0] + dir[0]), self.range_checker_vertical(self.current_node[1] + dir[1]))

    def _assign_dynamics(self):
        # have not discovered : 1
        # discovered : 0
        # not reachable : -1

        for i in range(-5, 6):
            for j in range(abs(i) - 5, 5 - abs(i) + 1):
                if self.map[self.range_checker_horizontal(
                        self.current_node[0] + i)][self.range_checker_vertical(self.current_node[1] + j)] == self.not_explored_weight:
                    self.map[self.range_checker_horizontal(
                        self.current_node[0] + i)][self.range_checker_vertical(self.current_node[1] + j)] = 0

        for thing in self.things:
            x = self.range_checker_horizontal(
                self.current_node[0] + thing["x"])
            y = self.range_checker_vertical(self.current_node[1] + thing["y"])
            if thing["type"] == "taskboard":
                self.task_boards.add((x, y))
            elif thing["type"] == "dispenser":
                d = (thing["details"], (x, y))
                self.dispensers.add(d)
            # elif thing["type"] == "block":
            #     d = (thing["details"], (x, y))
            #     self.blocks.add(d)
                # self.map[x][y] = self.obstacle_weigth

        if "obstacle" in self.terrains:
            for obstacle in self.terrains["obstacle"]:
                x = self.range_checker_horizontal(
                    self.current_node[0] + obstacle[0])
                y = self.range_checker_vertical(
                    self.current_node[1] + obstacle[1])
                self.obstacles.add((x, y))
                self.map[x][y] = self.obstacle_weigth

        if "goal" in self.terrains:
            for goal in self.terrains["goal"]:
                x = self.range_checker_horizontal(
                    self.current_node[0] + goal[0])
                y = self.range_checker_vertical(self.current_node[1] + goal[1])
                self.goals.add((x, y))

    def _target_not_found(self):
        directions = {0: (0, -1), 1: (-1, 0), 2: (0, 1), 3: (1, 0)}
        dir_str = {"n": [0, -1], "w": [-1, 0], "s": [0, 1], "e": [1, 0]}
        can_move = False
        rand_number = 0
        cnt = 0
        dic = self._explore(self.extra_vision)
        dir = directions[0]
        while(not can_move):
            obstacle_found = False
            rand_number = dic[cnt][0]
            dir = directions[rand_number]
            cnt += 1
            if self.seeking == 2:
                dis_loc = (dir_str[self.dis_pos][0] + dir[0],
                           dir_str[self.dis_pos][1] + dir[1])
            x4 = self.range_checker_horizontal(self.current_node[0] + dir[0])
            y4 = self.range_checker_vertical(self.current_node[1] + dir[1])
            for obstacle in self.obstacles:
                if(obstacle == (x4, y4)):
                    obstacle_found = True
                if(self.seeking == 2 and obstacle == dis_loc):
                    obstacle_found = True
            for block in self.blocks:
                if(block[1] == (x4, y4)):
                    obstacle_found = True
                if(self.seeking == 2 and block == dis_loc):
                    obstacle_found = True
            if(not obstacle_found):
                can_move = True
                self.last_dir = self.directions_char[rand_number]

        self.agent.move(self.directions_char[rand_number])
        self.current_node = (self.range_checker_horizontal(
            self.current_node[0] + dir[0]), self.range_checker_vertical(self.current_node[1] + dir[1]))

    def best_task(self):
        tasks = self.tasks
        task_name = tasks[0]["name"]
        dis_type = tasks[0]["requirements"]["type"]
        mx = tasks[0]["deadline"]
        for i in range(1, len(tasks)):
            if tasks[i]["deadline"] > mx:
                task_name = tasks[i]["name"]
                mx = tasks[i]["deadline"]
        # print(dis_type)
        # print("Heyyy---------")
        return task_name

    def dispenser_name(self, dir):
        directions = {"n": [0, -1], "w": [-1, 0], "s": [0, 1], "e": [1, 0]}
        for dispenser in self.dispensers:
            if dispenser[1] == (self.range_checker_horizontal(self.current_node[0] + directions[dir][0]), self.range_checker_vertical(self.current_node[1] + directions[dir][1])):
                return True, dispenser[0]
        return False, ""


breakout_room_id = 1  # type your breakout room id here!
team_name = 'A'+str(breakout_room_id)
entity_name = 'agent'+team_name
agent1 = mapc2020.Agent.open(user=entity_name, pw=str(breakout_room_id))

# entity, map size, initial node, extra_vision, not_explored_weight, obstacle_weigth
agent = Agent(agent1, (40, 40), (15, 10), 3, 1, -3)
cnt = 0
task_accepted = False
chosen_task_loc = ()
chosen_task_name = ""

while(True):
    agent._get_dynamics()
    agent._assign_dynamics()
    cnt += 1
    reachable = False
    min_depth = 1600
    answer = ''
    objects = []
    objects.append(agent.task_boards)
    objects.append(agent.dispensers)
    objects.append(agent.goals)
    dir_char = ["s", "e", "n", "w"]
    agent.exploration_map[agent.current_node[0]][agent.current_node[1]] = 1
    # Goal found in vision logic
    if(agent.seeking == 1):
        if(agent.found_dispenser):
            reach, min_depth, answer = agent.is_reachable(
                agent.requirement_pos)
            reachable = True
        else:
            for task in agent.tasks:
                if(task["name"] == agent.chosen_task_name):
                    for dispenser in agent.dispensers:
                        if(not agent.found_dispenser):
                            directions = {0: [0, -1],
                                          1: [-1, 0], 2: [0, 1], 3: [1, 0]}
                            for p in range(4):
                                x2 = agent.range_checker_horizontal(
                                    directions[p][0] + dispenser[1][0])
                                y2 = agent.range_checker_vertical(
                                    directions[p][1] + dispenser[1][1])
                                reach, mn, answer1 = agent.is_reachable(
                                    (x2, y2))
                                if(dispenser[0] == task["requirements"][0]["type"] and reach and mn < min_depth):
                                    min_depth = mn
                                    answer = answer1
                                    agent.found_dispenser = True
                                    agent.requirement_pos = (x2, y2)
                                    reachable = True
                                    agent.dis_pos = dir_char[p]
    else:
        for task_board in objects[agent.seeking]:
            if agent.seeking == 2:
                reach, depth, ans = agent.is_reachable_with_box(task_board)
            else:
                reach, depth, ans = agent.is_reachable(task_board)
            if(reach and depth < min_depth):
                reachable = True
                min_depth = depth
                answer = ans

    # Goal not found in vision logic
    # for k in range(len(agent.map)):
    #     for p in range(len(agent.map[k])):
    #         print(agent.map[p][k], end=" ")
    #     print()

    if(reachable):
        if(min_depth == 0):
            if(agent.seeking == 0):
                reach, min_depth, ans, chosen_task_name, requirement_pos, dis_pos = agent.solve_task()
                if(reach):
                    agent.found_dispenser = True
                    print("Accepted: ")
                    print(chosen_task_name)
                    agent.chosen_task_name = chosen_task_name
                    agent.agent.accept(agent.chosen_task_name)
                    dir_char = {"n": "s", "w": "e", "s": "n", "e": "w"}
                    agent.requirement_pos = requirement_pos
                    agent.dis_pos = dir_char[dis_pos]
                else:
                    if(len(agent.tasks) > 0):
                        print("Randomly accepted: ")
                        # print(agent.tasks[0]["name"])
                        # print(agent.agent.dynamic)
                        task_name = agent.best_task()
                        print(task_name)
                        agent.chosen_task_name = task_name
                        agent.agent.accept(agent.chosen_task_name)
                        agent._target_not_found()
                    else:
                        print("Skipping")
                        agent.agent.skip()
                agent.seeking = 1
            elif(agent.seeking == 1):
                dir_reverser = {(0, -1): "s", (-1, 0): "e",
                                (0, 1): "n", (1, 0): "w"}
                dir = agent.dis_pos
                agent.is_attached = True
                status, dispenser_name = agent.dispenser_name(dir)
                if status:
                    agent.block_detail = dispenser_name
                else:
                    print("Error with dispenser_name")
                print("Attaching box at: ")
                print(agent.current_node)
                print("With direction " + dir)
                agent.agent.request(dir)
                agent.agent.attach(dir)
                agent.seeking = 2
            else:
                try:
                    print("Submitting task at: ")
                    print(agent.current_node)
                    agent.is_attached = False
                    agent.agent.submit(agent.chosen_task_name)
                except mapc2020.AgentActionError:
                    directions = {"n": (0, -1), "w": (-1, 0),
                                  "s": (0, 1), "e": (1, 0)}
                    print("Could not submit task, detaching")
                    # print(agent.agent.dynamic)
                    agent.agent.detach(agent.dis_pos)
                    pos = (agent.range_checker_horizontal(
                        agent.current_node[0] + directions[agent.dis_pos][0]), agent.range_checker_vertical(agent.current_node[1] + directions[agent.dis_pos][1]))
                    d = (agent.block_detail, pos)
                    agent.blocks.add(d)
                    agent.map[pos[0]][pos[1]] = agent.obstacle_weigth
                    agent.is_attached = False
                    # print(agent.agent.dynamic)
                agent.found_dispenser = False
                agent.seeking = 0
        else:
            agent.agent_move(answer)
    else:
        agent._target_not_found()

print("Steps: ", cnt)
print("Current node: ", agent.current_node)
print("Goals: ", agent.goals)
print("Obstacles: ", agent.obstacles)
print("Dispensers: ", agent.dispensers)
print("Task boards: ", agent.task_boards)
