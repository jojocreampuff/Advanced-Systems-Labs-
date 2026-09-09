clc; clear; close all;

NStates = 6;
NOrder = 3;

f1 = 'kappa';
f2 = 'lambda';


disp(strcat("Creating ", f1, " and ", f2))
tic

f = '';
az = 'abcdefghijklmnopqrstuvwxyz';


r = cell(NOrder, 1);

for i = 1:NOrder
    r{i} = cmbd(NStates, i);
end

for i = 1:length(r)
    for j = 1:size(r{i})
        rx = r{i};
        y = rx(j, :);
        f = strcat(f, "          ", oper(y), "\n");
    end
end

f = strcat(f, "          ones(1, size(x1, 2))");

f = strcat(f, "\n");

header = '(';
for i = 1:(NStates - 1)
    header = strcat(header, az(i), ', ');
end
header = strcat(header, az(NStates), ')\n');

out = 'function y = %funcname%(x)\n';
out = strcat(out, '  y = %filename%(');
for i = 1:NStates
    out = strcat(out, 'x(', num2str(i), ', :)');
    if i < NStates 
        out = strcat(out, ', ');
    end
end
out = strcat(out, ");\nend\n\n");
out = strcat(out, "function out = ", "%filename%", header);
out = strcat(out, "  out = [\n", f, "\n  ];\nend");

for i = 1:NStates
    out = strrep(out, strcat('x', string(i)), az(i));
end



fileID = fopen(strcat(f1, '.m'), 'w');
out_rho = out;
out_rho = strrep(out_rho, "%filename%", strcat(f1, "mat"));
out_rho = strrep(out_rho, "%funcname%", f1);
fprintf(fileID, out_rho);
fclose(fileID);

fileID = fopen(strcat(f2, '.m'), 'w');
out_rho = out;
out_rho = strrep(out_rho, "%filename%", strcat(f2, "mat"));
out_rho = strrep(out_rho, "%funcname%", f2);
fprintf(fileID, out_rho);
fclose(fileID);


%% Create the Gradients
disp("Creating gradients")


% A 'recursive' generalization of the gradient of phi
x_sym = sym('x', [1 NStates], 'real')';
phi = @(x) fphi(x, f1);
NPhi = numel(phi(x_sym));

sym_phi = phi(x_sym);
gradlist_phi = cell(NPhi, NStates);

for i = 1:NPhi
    for j = 1:NStates
      gradlist_phi{i, j} = matlabFunction(diff(@(x) sym_phi(i), x_sym(j)), "Vars", {x_sym});
    end
    fprintf("Calculating Gradient: %f%%\n", 100*i/NPhi)
end


[m, n]  = size(gradlist_phi);


fun = strcat('function y = grad_', f1, '_%i%(x)\n');
fun = strcat(fun, '  y = grad_', f1, 'mat(');
for i = 1:NStates
    fun = strcat(fun, 'x(', num2str(i), ", :)");
    if i < NStates
        fun = strcat(fun, ', ');
    end
end
fun = strcat(fun, ")';\nend\n\n");

for i = 1:n
    
    out = strcat(fun, 'function out = grad_', f1, 'mat', header, '\n');
    out = strcat(out, "out = [\n");

    for j = 1:m
        s = eraseBetween(func2str(gradlist_phi{j, i}), 1, 6);

        for k = 1:NStates
            s = strrep(s, strcat('in1(', num2str(k), ',:)'), az(k));
        end

        s = strrep(s, '1.0', 'ones(1, length(a))');
        s = strrep(s, '0.0', 'zeros(1, length(a))');

        out = strcat(out, s, '\n');
    end

    out = strcat(out, "    ];\nend");
    out = strrep(out, "%i%", num2str(i));

    fileID = fopen(strcat('grad_', f1, '_', num2str(i), '.m'), 'w');
    fprintf(fileID, out);
    fclose(fileID);
end

toc


function y = cmbd(N, l)
    count = ones(1, l);
    y = ones(0,0);
    
    while true
        y(end + 1, :) = count;

        if isequal(count, N*ones(1, l))
            break 
        end

        count(end) = count(end) + 1; 
        count = checkMax(count, N, l);
    end
end

function t = checkMax(t, N, i)
  if t(i) > N && i ~= 1
      t(i - 1) = t(i - 1) + 1;
      t = checkMax(t, N, i - 1);
      t(i) = t(i - 1);
  end
end

function y = oper(arr)
    y = ['x' num2str(arr(1))];
    if numel(arr) ~= 1
        for i = arr(:, 2:end)
            y = strcat(y, '.*x', num2str(i));
        end
    end
end

function y = fphi(x, f1)
  y = feval(f1, x);
end

