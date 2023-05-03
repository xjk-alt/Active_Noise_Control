import msgpack
import numpy as np
import matplotlib.pyplot as plt
import argparse

if __name__ == '__main__':

    import argparse
    
    # 1. 定义命令行解析器对象
    parser = argparse.ArgumentParser(description='plot ANC experiment msgpack data')
    
    parser.add_argument('-f', '--file', type=str, default="output.bin", help="msgpack file")
    parser.add_argument('-xy' ,'--xy',nargs='*' ,type=str, help='X,Y variable to plot')
    
    args = parser.parse_args()

    file = args.file
    name = args.xy
    
    try:
    
        f = open(file,"rb")
        data = msgpack.load(f)
        f.close()

    except:
        raise Exception("Fail to open file")

    for var_name in name:
        x_name,y_name = var_name.split(",")
        
        try:
            x,y = data[x_name], data[y_name]
            plt.figure()
            plt.plot(x, y)
            plt.xlabel(x_name)
            plt.ylabel(y_name)
            plt.title(f"{x_name}-{y_name}")
        except KeyError:
            print(f"Fail to unpack X: {x_name}, Y: {y_name}")
    
    plt.show()
        
        
