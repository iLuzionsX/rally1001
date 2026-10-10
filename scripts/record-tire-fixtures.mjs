import {readFileSync,writeFileSync,mkdirSync,mkdtempSync,rmSync} from 'node:fs';
import {createRequire} from 'node:module';
import {dirname,join} from 'node:path';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';

// Module-level fixtures for porting the tire curves: CARTIRE outputs over a grid
// of loads, slips and friction, per surface. Output: reference/golden-traces/tires.json.
const root=dirname(dirname(fileURLToPath(import.meta.url))),cache=join(root,'node_modules/.cache');
mkdirSync(cache,{recursive:true});const temp=mkdtempSync(join(cache,'rally-tires-'));
const require=createRequire(import.meta.url);
try{
  for(const name of ['vendor/stunt-rally/gravel','vendor/stunt-rally/pacejka']){
    const {outputText}=ts.transpileModule(readFileSync(join(root,'lib/rally',name+'.ts'),'utf8'),{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022}});
    mkdirSync(dirname(join(temp,name+'.cjs')),{recursive:true});
    writeFileSync(join(temp,name+'.cjs'),outputText.replace(/require\("\.\/(.*?)"\)/g,'require("./$1.cjs")'));
  }
  const {TIRES,STUNT_RALLY_GRAVEL}=require(join(temp,'vendor/stunt-rally/pacejka.cjs'));
  const curves={...TIRES,stuntRally:STUNT_RALLY_GRAVEL},loads=[0,400,1500,2500,3700,4600,6000,9000,31000],mus=[.34,.59,1];
  const report={};
  for(const [name,curve] of Object.entries(curves)){
    const rows=[];
    for(const load of loads){
      const row={load,optimum:curve.optimumSlip(load),slipAtShare:[.5,.75,.9,.97,1].map(share=>({share,slip:curve.slipAtShare(load,share)})),forces:[]};
      for(const mu of mus){
        row.forces.push({mu,lateralPeak:curve.lateralPeak(load,mu),longitudinalPeak:curve.longitudinalPeak(load,mu),stiffness:curve.stiffness(load,mu)});
        for(const ratio of [-1,-.3,-.08,0,.02,.12,.4,1])for(const angle of [-.8,-.2,-.05,0,.01,.1,.3,1.2])
          row.forces.push({mu,ratio,angle,...curve.force(load,mu,ratio,angle)});
      }
      rows.push(row);
    }
    report[name]=rows;
  }
  mkdirSync(join(root,'reference/golden-traces'),{recursive:true});
  writeFileSync(join(root,'reference/golden-traces/tires.json'),JSON.stringify(report)+'\n');
  console.log(Object.keys(report).join(','),'->',loads.length*Object.keys(report).length,'load rows');
}finally{rmSync(temp,{recursive:true,force:true});}
