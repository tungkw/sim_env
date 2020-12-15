# object
- get object
    - simxGetObjectHandle           
    - simxGetObjectChild            
    - simxSetObjectParent           
    - simxGetObjects                
    - simxGetObjectsInTree             
- get object feature
    - simxGetObjectMatrix           
    - simxGetObjectOrientation      
    - simxGetObjectPose             
    - simxGetObjectPosition         
    - simxGetObjectQuaternion       
    - simxGetObjectParent           
    - simxGetObjectSelection        
    - simxGetObjectFloatParameter   
    - simxGetObjectIntParameter     
    - simxGetObjectStringParameter  
    - simxGetObjectVelocity          
    - simxGetObjectName         
- set object feature
    - simxSetObjectFloatParameter   
    - simxSetObjectIntParameter     
    - simxSetObjectMatrix           
    - simxSetObjectOrientation      
    - simxSetObjectPose             
    - simxSetObjectPosition         
    - simxSetObjectQuaternion       
    - simxSetObjectSelection        
    - simxSetObjectStringParameter  
- other operations
    - simxAddDrawingObject_cubes    
    - simxAddDrawingObject_points   
    - simxAddDrawingObject_segments 
    - simxAddDrawingObject_spheres  
    - simxAddDrawingObject_triangles 
    - simxCopyPasteObjects          
    - simxRemoveDrawingObject       
    - simxRemoveObjects             

# simulation
- simxStartSimulation           
- simxPauseSimulation           
- simxStopSimulation            
- topic or communication channel
    - simxServiceCall               
    - simxDefaultPublisher          
    - simxDefaultSubscriber         
    - simxCreatePublisher           
    - simxCreateSubscriber          
    - simxRemovePublisher           
    - simxRemoveSubscriber          
- simxGetServerTimeInMs         
- simxGetSimulationState        
- simxGetSimulationTime         
- simxGetSimulationTimeStep     
- simxGetTimeInMs               
- simxLoadModelFromBuffer       
- simxLoadModelFromFile         
- simxLoadScene                 
- simxCloseScene                
- simxSleep                     
- sychronous
    - simxSynchronous               
    - simxSynchronousTrigger        
    - simxGetSimulationStepDone     
    - simxGetSimulationStepStarted  
    - simxSpin                      
    - simxSpinOnce                  


# GUI
- simxAddStatusbarMessage       
- simxAuxiliaryConsoleClose     
- simxAuxiliaryConsoleOpen      
- simxAuxiliaryConsolePrint     
- simxAuxiliaryConsoleShow      
- simxDisplayDialog             
- simxEndDialog                  
- simxGetDialogInput            
- simxGetDialogResult           

# force sensor
- simxReadForceSensor           
- simxBreakForceSensor

# vision sensor
- simxCheckVisionSensor         
- simxGetVisionSensorDepthBuffer
- simxGetVisionSensorImage      
- simxSetVisionSensorImage      
- simxReadVisionSensor          

# proximity sensor
- simxCheckProximitySensor      
- simxReadProximitySensor       

# script          
- simxCallScriptFunction        
- simxExecuteScriptString       
- simxEvaluateToInt             
- simxEvaluateToStr          

# signal
- simxGetFloatSignal            
- simxGetIntSignal              
- simxGetStringSignal           
- simxSetFloatSignal            
- simxSetIntSignal              
- simxSetStringSignal           
- simxClearFloatSignal   
- simxClearIntegerSignal        
- simxClearStringSignal          

# parameter
- simxGetIntParameter           
- simxGetStringParameter        
- simxGetArrayParameter         
- simxGetBoolParameter    
- simxGetFloatParameter         
- simxSetIntParameter           
- simxSetArrayParameter         
- simxSetBoolParameter          
- simxSetFloatParameter         
- simxSetStringParameter        

# joint
- simxGetJointForce             
- simxGetJointMaxForce          
- simxGetJointPosition          
- simxGetJointTargetPosition    
- simxGetJointTargetVelocity    
- simxSetJointForce             
- simxSetJointMaxForce          
- simxSetJointPosition          
- simxSetJointTargetPosition    
- simxSetJointTargetVelocity    

# others
- simxCreateDummy             
- simxCheckCollision            
- simxCheckDistance             
- simxGetCollectionHandle       
- simxGetCollisionHandle        
- simxGetDistanceHandle         
- simxReadCollision             
- simxReadDistance              
