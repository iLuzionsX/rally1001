import type RAPIER from '@dimforge/rapier3d-compat';
import type {RallyVehicle,DriveInput} from './vehicle';

// Like Stunt Rally's CARDYNAMICS::Tick, solve driveline, tires and body in small
// fixed steps. Rendering, race timing and particles retain their 60 Hz cadence.
export const PHYSICS_STEP=1/240;
export const DRIVELINE_STEP=1/960;
/** Accumulated wall time (ms) in the vehicle model and in Rapier's world step; the engine samples and clears it. */
export const SIM_TIMINGS={car:0,world:0};
export function advanceVehicle(vehicle:RallyVehicle,world:RAPIER.World,input:DriveInput,dt:number,enabled:boolean){
  const previousPosition={...vehicle.position},previousRotation={...vehicle.rotation};
  const count=Math.max(1,Math.ceil(dt/PHYSICS_STEP-1e-8)),step=dt/count;
  world.timestep=step;
  for(let i=0;i<count;i++){
    let t=performance.now();
    vehicle.beforeStep(input,step,enabled);
    const t1=performance.now();SIM_TIMINGS.car+=t1-t;t=t1;
    world.step();
    const t2=performance.now();SIM_TIMINGS.world+=t2-t;t=t2;
    vehicle.afterStep(step);
    SIM_TIMINGS.car+=performance.now()-t;
  }
  // Interpolation spans the entire visual tick, rather than only its last substep.
  vehicle.previousPosition=previousPosition;vehicle.previousRotation=previousRotation;
}
