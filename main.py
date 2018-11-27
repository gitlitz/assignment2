#!/usr/bin/python
usage = """Enter task number:
1 - Move forward
2 - Turn around
3 - Distance to object with color X
4 - Find object with color X"""

def move_forward():
    pass

def turn_around():
    pass

def distance_to_color():
    pass

def find_object():
    pass

def main():
    tasks_dict = {"1": move_forward,
            "2": turn_around,
            "3": distance_to_color,
            "4": find_object
            }

    while True:
        print(usage)
        txt = raw_input()
        if txt in tasks_dict:
            tasks_dict[txt]()
        else:
            print("Error: invalid task " + txt)


if __name__ == '__main__':
    main()
