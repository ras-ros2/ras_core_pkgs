def ras_print(*args,**kwargs):
    kwargs["flush"] = True
    print(args,kwargs)