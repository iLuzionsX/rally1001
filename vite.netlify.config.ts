import {defineConfig} from 'vite';
import react from '@vitejs/plugin-react';
import {fileURLToPath} from 'node:url';
import path from 'node:path';

const here=path.dirname(fileURLToPath(import.meta.url));
export default defineConfig({
  root:path.resolve(here,'netlify-static'),
  publicDir:path.resolve(here,'public'),
  resolve:{alias:{'@':here}},
  plugins:[react()],
  build:{
    outDir:path.resolve(here,'dist-netlify'),
    emptyOutDir:true,
    target:'es2022',
    chunkSizeWarningLimit:1600,
  },
});
