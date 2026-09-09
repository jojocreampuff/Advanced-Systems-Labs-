function output=One_hidden_layer_feedforward(input,W1,W2,b1,b2,mean_inputs,std_inputs,mean_outputs,std_outputs)
%Accidentally, the first layer is sigmoid and second is tansig
input_data = (input - mean_inputs) ./ std_inputs;
% First hidden layer with log-sigmoid activation function
z1 = W1 * input_data + b1;         
a1 = logsig(z1);                   
% Second hidden layer with tansig 
z2 = W2 * a1 + b2;                                                  
output_data = z2;                 

output = output_data .* std_outputs + mean_outputs;
