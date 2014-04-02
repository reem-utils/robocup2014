import rospy


LOCATIONS = rospy.get_param('/robocup_params/locations')
ITEMS = rospy.get_param('/robocup_params/items')
NAMES = rospy.get_param('/robocup_params/persons')
CATEGORYS = rospy.get_param('/robocup_params/it_category')
PERSONS = NAMES

# locations = 'loc'
# items = 'it'
# print dir()
# locations =['a', 's', 'd', 'f']
# items = ['q', 'w', 'e', 'r']

def obj2idx(Object, List):
    return globals()[List].index(Object)

def idx2obj(Index, List):
    return globals()[List][Index]

def get_list(List):
    # print List
    return globals()[List]

def get_category_list(Category, List='it_category'):
    string = '/robocup_params/' + List + '/' + Category
    return rospy.get_param(string)
def get_loc_category_list(Category, List='loc_category'):
    string = '/robocup_params/' + List + '/' + Category
    return rospy.get_param(string)

def get_obj_location(Object):
    dic = rospy.get_param('/robocup_params/location')
    found = False
    for loc in dic:
        if Object in dic[loc]:
            return loc
            found = True
    if not found:
        return 'NULL'



# def get_pos(name):
#     rospy.get_param('/coord_translator/')