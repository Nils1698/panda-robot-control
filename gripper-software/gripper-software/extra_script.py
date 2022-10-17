Import("env")

def after_upload(source, target, env):
    import time
    import os
    i=0
    port = env.GetProjectOption("monitor_port")
    print("Wait for '"+port+"' to become available again...")
    while(os.path.exists(port) == False):
        time.sleep(0.01)
        if(i>=500): break
        else: i+=1
    if(i>=500):
        print(".. timed out")
    else:
        print("'"+port+"' is ready")

env.AddPostAction("upload", after_upload)
