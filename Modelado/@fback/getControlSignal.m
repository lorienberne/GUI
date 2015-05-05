function signal = getControlSignal(this,state, desiredState)
    signal = -this.K*(state - desiredState);
end
