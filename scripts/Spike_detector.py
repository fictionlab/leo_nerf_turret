from collections import deque

class Detector:
    
    def __init__(self) -> None:
        pass
    
    def __init__(self, threshold_min, threshold_max, delay_samples = 10, debug_mode = False)-> None:
        
        self.threshold_min = threshold_min
        self.threshold_max = threshold_max
        
        self.sample_number = delay_samples         
        self.delay_samples = delay_samples
        
        self.last_sample_value = 0
        
        self.spike_up = False
        self.spike_down = False
        
        self.debug_mode = debug_mode
        self.sample_buffer = deque([],delay_samples*2)
        self.last_hit_data = None
        
        print("Oh shit whattup?")
        
        
    def check(self, sample):
        spike = False
        
        self.sample_buffer.append(sample)
        
        #Setup the values
        if self.spike_down or self.spike_up:
            self.spike_down = False
            self.spike_up = False
            
        
        #Skip looking for spikes if in delay frames    
        if self.sample_number < self.delay_samples:
            self.sample_number += 1
            self.last_sample_value = sample
            if self.sample_number == self.delay_samples and self.debug_mode:
                self.last_hit_data = self.sample_buffer
            return False
            
        
        #Check if crossing upper threshold value while going up
        if sample >= self.threshold_max and self.last_sample_value < self.threshold_max:
            self.sample_number = 0
            spike = True
            self.spike_up = True
            self.check_buffer(down=True)
        #Same thing for going down and lower threshold
        elif sample <= self.threshold_min and self.last_sample_value > self.threshold_min:
            self.sample_number = 0
            spike = True
            self.spike_down = True
            self.check_buffer(up=True)
            
        self.last_sample_value = sample
        return spike
    
    
    
    def check_buffer(self, up = False, down = False):
        if not (up or down):
            return False
        
        for x in self.sample_buffer:
            if up and x >= self.threshold_max*0.85:
                self.spike_up = True
                return True
                
            #Same thing for going down and lower threshold
            elif down and x <= self.threshold_min*0.85:
                self.spike_down = True
                return True