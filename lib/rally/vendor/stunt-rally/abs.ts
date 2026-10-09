/*!
 * Stunt Rally / VDrift CARDYNAMICS::DoABS adapter.
 * Copyright Stunt Rally and VDrift contributors, GPL-3.0-or-later.
 * Modified: C++ -> TypeScript; pass contact slip/rotation and bypass handbrake;
 * the target slip comes from the curve of the surface under the wheel.
 * Pinned original: reference/stunt-rally/cardynamics_simulate.cpp, lines 844-881.
 */
import type {TireCurve} from './pacejka';
export class TireABS {
  active=false;
  update(demand:number,slip:number,load:number,maxWheelSpeed:number,tire:TireCurve){
    if(demand>.1&&maxWheelSpeed>6){
      const {sigmaHat}=tire.optimumSlip(load),error=-slip-sigmaHat;
      if(error>0&&!this.active)this.active=true;
      if(error<-sigmaHat/2&&this.active)this.active=false;
    }else this.active=false;
    return this.active?0:demand;
  }
}
