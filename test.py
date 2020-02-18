f = open("a.out").readlines()

for i in f:
    i= i.replace("(","").replace(")","").replace(" ","").replace("\n","")
    print(i+",0")