# heap q(prio)
import heapq
import csv
import math
# Default dict to store
from collections import defaultdict, deque
import time

# Load cities from file
def load_cities(filename):
    cities = {}
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            city, lat, lon = row[0], float(row[1]), float(row[2])
            cities[city] = (lat, lon)
    return cities

# Load adjacencies into a bidirectional graph
def load_adjacencies(filename):
    graph = defaultdict(list)
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            city1, city2 = row
            graph[city1].append(city2)
            graph[city2].append(city1)  # Ensure symmetry
    return graph

cities = load_cities('coordinates.csv')
graph = load_adjacencies('adjacencies.txt')

# Haversine formula to calculate distances between coordinates
def haversine(coord1, coord2):
    R = 6371  # Radius of the Earth in kilometers
    lat1, lon1 = math.radians(coord1[0]), math.radians(coord1[1])
    lat2, lon2 = math.radians(coord2[0]), math.radians(coord2[1])

    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

# DFS Search
def dfs(graph, start, goal, path=None):
    if path is None:
        path = [start]
    if start == goal:
        return path
    for neighbor in graph[start]:
        if neighbor not in path:
            new_path = dfs(graph, neighbor, goal, path + [neighbor])
            if new_path:
                return new_path
    return None

# BFS Search
def bfs(graph, start, goal):
    queue = deque([(start, [start])])
    while queue:
        vertex, path = queue.popleft()
        for neighbor in graph[vertex]:
            if neighbor not in path:
                if neighbor == goal:
                    return path + [neighbor]
                queue.append((neighbor, path + [neighbor]))
    return None

# ID-DFS Search
def id_dfs(graph, start, goal, max_depth):
    def dls(node, depth):
        if depth == 0 and node == goal:
            return [node]
        if depth > 0:
            for neighbor in graph[node]:
                path = dls(neighbor, depth - 1)
                if path:
                    return [node] + path
        return None

    for depth in range(max_depth):
        result = dls(start, depth)
        if result:
            return result
    return None

# Best-First Search
def best_first_search(graph, cities, start, goal):
    queue = [(0, start, [start])]
    visited = set()

    while queue:
        cost, node, path = heapq.heappop(queue)
        if node in visited:
            continue
        visited.add(node)

        if node == goal:
            return path

        for neighbor in graph.get(node, []):
            if neighbor not in cities:
                continue
            heapq.heappush(queue, (haversine(cities[node], cities[neighbor]), neighbor, path + [neighbor]))

    return None

# A* Search
def a_star_search(graph, cities, start, goal):
    queue = [(0, start, [start], 0)]
    visited = set()

    while queue:
        f, node, path, g = heapq.heappop(queue)
        if node in visited:
            continue
        visited.add(node)

        if node == goal:
            return path

        for neighbor in graph.get(node, []):
            if neighbor not in cities:
                continue
            g_new = g + haversine(cities[node], cities[neighbor])
            f_new = g_new + haversine(cities[neighbor], cities[goal])
            heapq.heappush(queue, (f_new, neighbor, path + [neighbor], g_new))

    return None

# Calculate total distance of the path
def calculate_total_distance(path, cities):
    total_distance = 0
    for i in range(len(path) - 1):
        total_distance += haversine(cities[path[i]], cities[path[i + 1]])
    return total_distance

# Main function with error handling and time measurement
def main():
    while True:
        start = input("Enter the starting city: ")
        goal = input("Enter the destination city: ")

        if start not in cities or goal not in cities:
            print("Invalid city names. Please try again.")
            continue

        print("Select a search method:")
        print("1. DFS")
        print("2. BFS")
        print("3. ID-DFS")
        print("4. Best-First Search")
        print("5. A* Search")
        choice = input("Enter your choice (1-5): ")

        search_methods = {
            '1': lambda: dfs(graph, start, goal),
            '2': lambda: bfs(graph, start, goal),
            '3': lambda: id_dfs(graph, start, goal, 10),
            '4': lambda: best_first_search(graph, cities, start, goal),
            '5': lambda: a_star_search(graph, cities, start, goal)
        }

        if choice in search_methods:
            start_time = time.time()

            try:
                result = search_methods[choice]()
                end_time = time.time()

                if result:
                    total_distance = calculate_total_distance(result, cities)
                    print(f"Path found: {' -> '.join(result)}")
                    print(f"Total distance: {total_distance:.2f} km")
                    print(f"Time taken: {end_time - start_time:.4f} seconds")
                else:
                    print("No path found.")

            except Exception as e:
                print(f"An error occurred: {e}")

        else:
            print("Invalid choice. Please try again.")

        if input("Try another search? (y/n): ").lower() != 'y':
            break

if __name__ == "__main__":
    main()
