function [net, tr, e, performance] = att_deep_network(net, x , t)

    % if isempty(net)
    %     hiddenLayerSize = [12, 24, 12, 6];
    %     net = fitnet(hiddenLayerSize, 'trainbr');
    % end
    
    % Setup Division of Data for Training, Validation, Testing
    net.divideParam.trainRatio = 80/100;
    net.divideParam.valRatio = 10/100;
    net.divideParam.testRatio = 10/100;
    
    % Train the Network
    [net,tr] = train(net,x,t,'useParallel','yes', 'useGPU','yes');
    
    % Test the Network
    y = net(x);
    e = gsubtract(t,y);
    performance = perform(net,t,y);
    
    % View the Network
    % view(net)
    
    % Plots
    % Uncomment these lines to enable various plots.
    %figure, plotperform(tr)
    %figure, plottrainstate(tr)
    %figure, ploterrhist(e)
    %figure, plotregression(t,y)
    %figure, plotfit(net,x,t)

