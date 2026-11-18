function blkStruct = slblocks
    % slblocks
    % Registers 'EdgeImpulse_Library.slx' in the Simulink Library Browser
    % under the name 'Edge Impulse TinyML'.

    Browser.Library = 'EdgeImpulse_Library';
    Browser.Name    = 'Edge Impulse TinyML';
    Browser.IsFlat  = 1;

    blkStruct.Browser = Browser;
end
