f = cell(1000, 1);
for i = 1:10
    f{i} = strcat('With Wn (', num2str(i),'):');
    disp(f{i});
    [os, st] = itaeosts(i);
    f{i} = strcat('Overshoot: ', num2str(os));
    disp(f{i});
    f{i} = strcat('Settling Time: ', num2str(st));
    disp(f{i});
    disp(' ');
end