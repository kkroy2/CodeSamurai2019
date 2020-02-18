file = open("prob2out.txt").readlines()
out_kml = open("route2.kml", "w")
out_description = open("description2.txt", "w")
out_description.write("Problem No: 2\n")

kmlheader = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://earth.google.com/kml/2.1\">\n<Document>\n<Placemark>\n<name>route.kml</name>\n<LineString>\n<tessellate>1</tessellate>\n<coordinates>\n"
kmlfooter = "</coordinates>\n</LineString>\n</Placemark>\n</Document>\n</kml>"
src, dest = 0, 0
srclat, srclng, destlat, destlng = 0.0 , 0.0, 0.0, 0.0
for i in range(len(file)):
    line = file[i].replace("\n", "")
    if i == 0:
        temp = line.split(" ")
        src = int(temp[0])
        dest = int(temp[1])
    elif i == 1:
        temp = line.split(" ")
        srclat, srclng = temp[0], temp[1]
        out_description.write("Source: ("+ srclat+ ", "+ srclat+ ")\n")
    elif i == 2:
        temp = line.split(" ")
        destlat, destlng = temp[0], temp[1]
        out_description.write("Destination: ("+ destlat+ ", "+ destlat+ ")\n")

    else:
        temp = line.split(",")
        if i == 3:
            out_kml.write(kmlheader)
            if src != 1:
                out_kml.write(srclat + "," + srclng + ",0" + "\n")
            if src == 2:
                temp = line.split(",")
                out_description.write("Ride Car from ("+srclat+", "+srclng+") to ("+temp[0]+", "+temp[1]+").\n")
            if src == 3:
                temp = line.split(",")
                out_description.write("Walk from Source ("+srclat+", "+srclng+") to ("+temp[0]+", "+temp[1]+").\n")
        if i+1 != len(file):
            next = file[i+1].replace("\n", "").split(",")
            if next[3] == '1':
                out_description.write("Ride Metro from (" + temp[0] + ", " + temp[1] + ") to (" + next[0] + ", " + next[1] + ").\n")
            else:
                out_description.write("Ride Car from (" + temp[0] + ", " + temp[1] + ") to (" + next[0] + ", " + next[1] + ").\n")
        out_kml.write(temp[0]+","+temp[1] + ",0\n")
temp = file[len(file)-1].split(",")
if dest != 1:
    out_kml.write(destlat + "," + destlng + ",0" + "\n")
if dest == 2:
    out_description.write("Ride Car from (" + temp[0] + ", " + temp[1] + ") to (" + destlat + ", " + destlng + ").\n")
if dest == 3:
    out_description.write("Walk from Source (" + temp[0] + ", " + temp[1] + ") to (" + destlat + ", " + destlng + ").\n")
out_kml.write(kmlfooter)