function [b,a] =  butter1st_200hz(fc)
b = [1, 1] * 6283*fc / (400000 + 6283*fc);
a = [1 , -(400000 - 6283 *fc) / (400000 + 6283* fc)];
end