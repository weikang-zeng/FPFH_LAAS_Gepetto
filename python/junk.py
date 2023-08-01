import numpy as np
import open3d as o3d

class Header:
    def __init__(self):
        self.version = ""
        self.fields = []
        self.size = []
        self.type = []
        self.count = []
        self.width = -1
        self.height = -1
        self.viewpoint = []
        self.points = -1
        self.data = ""
        self.data_size = -1
    
    def is_complete(self):
        return ((self.version != "")
            and (self.fields != [])
            and (self.size != [])
            and (self.type != [])
            and (self.count != [])
            and (self.width != -1)
            and (self.height != -1)
            and (self.viewpoint != [])
            and (self.points != -1)
            and (self.data != "")
            and (self.data_size != -1))

    def whats_missing(self):
        if(self.is_complete()):
            print("Nothing is missing in this header")
        if(self.version == ""):
            print("VERSION is missing")
        if(self.fields == []):
            print("FIELDS is missing")
        if(self.size == []):
            print("SIZE is missing")
        if(self.type == []):
            print("TYPE is missing")
        if(self.count == []):
            print("COUNT is missing")
        if(self.width == -1):
            print("WIDTH is missing")
        if(self.height == -1):
            print("HEIGHT is missing")
        if(self.viewpoint == []):
            print("VIEWPOINT is missing")
        if(self.points == -1):
            print("POINTS is missing")
        if(self.data == ""):
            print("DATA is missing")

    def print(self):
        print("Version: \n\t%s" % self.version)
        furprint = ""
        for furprinti in self.fields:
            furprint+=furprinti+" "
        print("Fields: \n\t%s" % furprint)
        furprint = ""
        for furprinti in self.size:
            furprint+=str(furprinti)+" "
        print("Size: \n\t%s" % furprint)
        furprint = ""
        for furprinti in self.type:
            furprint+=furprinti+" "
        print("Type: \n\t%s" % furprint)
        furprint = ""
        for furprinti in self.count:
            furprint+=str(furprinti)+" "
        print("Count: \n\t%s" % furprint)
        print("Width: \n\t%i" % self.width)
        print("Heigth: \n\t%i" % self.height)
        furprint = ""
        for furprinti in self.viewpoint:
            furprint+=str(furprinti)+" "
        print("Viewpoint: \n\t%s" % furprint)
        print("Points: \n\t%i" % self.points)
        print("Data: \n\t%s" % self.data)


def read_header(file) -> Header:
    header = Header()
    while not header.is_complete():
        line = file.readline()
        print("line: %s"%line)
        head = line[0:line.find(' ')]
        info = line[line.find(' ')+1:line.find('\n')]
        if(head == "VERSION"):
            header.version = info
            continue
        if(head == "FIELDS"):
            while info.find(' ') != -1:
                header.fields.append(info[0:info.find(' ')])
                info = info[info.find(' ')+1:]
            header.fields.append(info)
            continue
        if(head == "SIZE"):
            while info.find(' ') != -1:
                header.size.append(int(info[0:info.find(' ')]))
                info = info[info.find(' ')+1:]
            header.size.append(int(info))
            continue
        if(head == "TYPE"):
            while info.find(' ') != -1:
                header.fields.append(info[0:info.find(' ')])
                info = info[info.find(' ')+1:]
            header.type.append(info)
            continue
        if(head == "COUNT"):
            while info.find(' ') != -1:
                header.count.append(int(info[0:info.find(' ')]))
                info = info[info.find(' ')+1:]
            header.count.append(int(info))
            header.data_size = int(0)
            for counter in header.count:
                header.data_size += int(counter)
            continue
        if(head == "WIDTH"):
            header.width = int(info)
            continue
        if(head == "HEIGHT"):
            header.height = int(info)
            continue
        if(head == "VIEWPOINT"):
            while info.find(' ') != -1:
                header.viewpoint.append(float(info[0:info.find(' ')]))
                info = info[info.find(' ')+1:]
            header.viewpoint.append(float(info))
            continue
        if(head == "POINTS"):
            header.points = int(info)
            continue
        if(head == "DATA"):
            header.data = info
            continue
    return header

def pcl_fpfh_to_o3d_feature(filename:str) -> o3d.pipelines.registration.Feature:
    fpfh = o3d.pipelines.registration.Feature
    file = open(filename, 'r')
    header = read_header(file)
    fpfh.resize(header.count, header.height*header.width)
    

