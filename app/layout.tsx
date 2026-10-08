import type {Metadata,Viewport} from 'next';
import './globals.css';
export const metadata:Metadata={title:'Wildtrail Rally — Canopy Loop',description:'A private jungle rally time trial. Drive a Subaru WRX STI or Ford F-150 through a textured jungle circuit under a photographed cloud sky, with suspension physics and personal-best timing.',icons:{icon:'/favicon.svg',shortcut:'/favicon.svg'},appleWebApp:{capable:true,statusBarStyle:'black-translucent',title:'Wildtrail Rally'}};
export const viewport:Viewport={width:'device-width',initialScale:1,maximumScale:1,userScalable:false,viewportFit:'cover',themeColor:'#101111'};
export default function RootLayout({children}:{children:React.ReactNode}){return <html lang="en"><body>{children}</body></html>;}
