function vnew = safe_normed_vector(v)
if norm(v) < 0.001
    vnew =  [1 0 0];
    return;
end
vnew  = v / norm(v);
end