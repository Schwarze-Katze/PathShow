import os
import yaml


def catAgentsPath(dic: dict, Paths: dict):
    scheduleTmp = dic['schedule']
    for key, value in scheduleTmp.items():
        if not key in Paths:
            Paths[key] = []
        Paths[key].extend(value)


curPath = os.path.dirname(os.path.realpath(__file__))
outputPath = os.path.join(curPath, "result_60*40")  # src/output/result_60*40
resultPaths = sorted(os.listdir(outputPath))  # (src/output/result_60*40/) ...
for resultPath in resultPaths:  # (src/output/result_60*40/) result1
    resultPath = outputPath+'/'+resultPath
    # (src/output/result_60*40/result1/)...
    yamlPaths = sorted(os.listdir(resultPath))
    Paths = {}
    for yamlPath in yamlPaths:  # src/output/result_60*40/result1/1.yaml
        with open(resultPath+'/'+yamlPath, 'r', encoding='utf-8') as f:
            config = f.read()
        dicTmp = yaml.safe_load(config)
        catAgentsPath(dicTmp,Paths)
    scheduleOutput={'schedule':Paths}
    outTmp=yaml.safe_dump(scheduleOutput)
    with open(resultPath+'/catOut.yaml', 'w', encoding='utf-8')as f:
        f.write(outTmp)

