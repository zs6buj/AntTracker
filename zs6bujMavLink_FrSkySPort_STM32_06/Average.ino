uint16_t Volt_AverageBuffer[10]; 
uint16_t Current_AverageBuffer[10]; 
  
//returns the average of Voltage for the 10 last values  
uint32_t Get_Volt_Average(uint16_t value)  {
      uint8_t i;
      uint32_t sum=0;
      
      for(i=9;i>0;i--)  {
          Volt_AverageBuffer[i]=Volt_AverageBuffer[i-1];
          sum+=Volt_AverageBuffer[i];
          }
      Volt_AverageBuffer[0]=value;    
      return (sum+=value)/10;
  }
  
  //returns the average of Current for the 10 last values  
uint32_t Get_Current_Average(uint16_t value)  {
      uint8_t i;
      uint32_t sum=0;
      
      for(i=9;i>0;i--)  {
          Current_AverageBuffer[i]=Current_AverageBuffer[i-1];
          sum+=Current_AverageBuffer[i];
          }
      Current_AverageBuffer[0]=value;    
      return (sum+=value)/10;
  }
