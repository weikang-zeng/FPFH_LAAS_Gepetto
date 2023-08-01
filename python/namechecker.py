import os

filePath = "/home/wzeng/multiscale-fpfh/output/plateforms_wo_sensor_origin/features"
for filename in os.walk(filePath):
    result=filename[2]
    keywords = result.split('_')
    print(keywords)
    