function f = float_constrain(v,low,up)
    if v>up
        f = up;
        return;
    end
    if v < low
        f = low;
        return
    end
    f = v;
end