import React from 'react';
import {createRoot} from 'react-dom/client';
import RaceGame from '../components/race-game';
import '../app/globals.css';

const root=document.getElementById('root');
if(!root)throw new Error('Missing rally root');
createRoot(root).render(<RaceGame/>);
