"use strict";(self.webpackChunkwebsite=self.webpackChunkwebsite||[]).push([[8150],{67651:(e,t,s)=>{s.r(t),s.d(t,{assets:()=>l,contentTitle:()=>i,default:()=>m,frontMatter:()=>o,metadata:()=>a,toc:()=>c});const a=JSON.parse('{"id":"data_formats/mps/slam/mps_multi_slam","title":"Multi-SLAM","description":"Multi-SLAM is a Project Aria Machine Perception Service (MPS) that can be requested on two or more recordings. It creates SLAM MPS outputs in a shared co-ordinate frame.","source":"@site/docs/data_formats/mps/slam/mps_multi_slam.mdx","sourceDirName":"data_formats/mps/slam","slug":"/data_formats/mps/slam/mps_multi_slam","permalink":"/projectaria_tools/docs/data_formats/mps/slam/mps_multi_slam","draft":false,"unlisted":false,"editUrl":"https://github.com/facebookresearch/projectaria_tools/tree/main/website/docs/data_formats/mps/slam/mps_multi_slam.mdx","tags":[],"version":"current","sidebarPosition":1000,"frontMatter":{"sidebar_position":1000,"title":"Multi-SLAM"},"sidebar":"tutorialSidebar","previous":{"title":"Calibration Data","permalink":"/projectaria_tools/docs/data_formats/mps/slam/mps_calibration"},"next":{"title":"Eye Gaze","permalink":"/projectaria_tools/docs/data_formats/mps/mps_eye_gaze"}}');var n=s(74848),r=s(28453);const o={sidebar_position:1e3,title:"Multi-SLAM"},i=void 0,l={},c=[];function d(e){const t={a:"a",code:"code",li:"li",p:"p",pre:"pre",ul:"ul",...(0,r.R)(),...e.components};return(0,n.jsxs)(n.Fragment,{children:[(0,n.jsxs)(t.p,{children:["Multi-SLAM is a Project Aria ",(0,n.jsx)(t.a,{href:"/projectaria_tools/docs/ARK/mps/",children:"Machine Perception Service (MPS)"})," that can be requested on two or more recordings. It creates SLAM MPS outputs in a shared co-ordinate frame."]}),"\n",(0,n.jsxs)(t.p,{children:["Multi-SLAM data can be visualized in the Python version of ",(0,n.jsx)(t.a,{href:"/projectaria_tools/docs/data_utilities/visualization/visualization_python#multi-slam",children:"MPS Viewer"}),"."]}),"\n",(0,n.jsx)(t.p,{children:"Open datasets that contain Multi-SLAM outputs where there are recordings with 2 or more Project Aria glasses:"}),"\n",(0,n.jsxs)(t.ul,{children:["\n",(0,n.jsx)(t.li,{children:(0,n.jsx)(t.a,{href:"/projectaria_tools/docs/open_datasets/aria_everyday_activities_dataset/",children:"Aria Everyday Activities (AEA) dataset"})}),"\n",(0,n.jsx)(t.li,{children:(0,n.jsx)(t.a,{href:"/projectaria_tools/docs/open_datasets/aria_digital_twin_dataset/",children:"Aria Digital Twin (ADT) dataset"})}),"\n"]}),"\n",(0,n.jsxs)(t.p,{children:["The Multi-SLAM outputs are mostly the same as the standard ",(0,n.jsx)(t.a,{href:"/projectaria_tools/docs/data_formats/mps/slam/",children:"SLAM MPS outputs"}),". The differences are:"]}),"\n",(0,n.jsxs)(t.ul,{children:["\n",(0,n.jsx)(t.li,{children:"Multi-SLAM can only be requested via MPS CLI"}),"\n",(0,n.jsxs)(t.li,{children:["All the recordings that were aligned together will have the same ",(0,n.jsx)(t.code,{children:"graph_uid"})," in the output."]}),"\n",(0,n.jsx)(t.li,{children:"The output may contain multiple aligned islands and multiple consecutive graph_uids."}),"\n"]}),"\n",(0,n.jsx)(t.p,{children:"Outputs are saved to a user defined directory. Each numbered folder contains the outputs for a specific VRS file:"}),"\n",(0,n.jsx)(t.pre,{children:(0,n.jsx)(t.code,{children:"\u2514\u2500\u2500 multi_slam_output # user defined directory for outputs\n    \u251c\u2500\u2500 0\n    \u2502   \u251c\u2500\u2500 slam\n    \u2502   \u2502    \u251c\u2500\u2500 closed_loop_trajectory.csv\n    \u2502   \u2502    \u251c\u2500\u2500 online_calibration.jsonl\n    \u2502   \u2502    \u251c\u2500\u2500 open_loop_trajectory.csv\n    \u2502   \u2502    \u251c\u2500\u2500 semidense_observations.csv.gz\n    \u2502   \u2502    \u251c\u2500\u2500 semidense_points.csv.gz\n    \u2502   \u2502    \u2514\u2500\u2500 summary.json\n    \u2502   \u251c\u2500\u2500 vrs_health_check.json\n    \u2502   \u2514\u2500\u2500 vrs_health_check_slam.json\n    \u251c\u2500\u2500 1\n    \u2502   \u251c\u2500\u2500 slam\n    \u2502   \u2502    \u251c\u2500\u2500 closed_loop_trajectory.csv\n    \u2502   \u2502    \u251c\u2500\u2500 online_calibration.jsonl\n    \u2502   \u2502    \u251c\u2500\u2500 open_loop_trajectory.csv\n    \u2502   \u2502    \u251c\u2500\u2500 semidense_observations.csv.gz\n    \u2502   \u2502    \u251c\u2500\u2500 semidense_points.csv.gz\n    \u2502   \u2502    \u2514\u2500\u2500 summary.json\n    \u2502   \u251c\u2500\u2500 vrs_health_check.json\n    \u2502   \u2514\u2500\u2500 vrs_health_check_slam.json\n    \u2514\u2500\u2500 vrs_to_multi_slam.json\n"})}),"\n",(0,n.jsxs)(t.p,{children:[(0,n.jsx)(t.code,{children:"vrs_to_multi_slam.json"})," associates the VRS file name with a numbered folder, for example:"]}),"\n",(0,n.jsx)(t.pre,{children:(0,n.jsx)(t.code,{children:'{\n  "/example/recording1.vrs": "0",\n  "/example/recording2.vrs": "1",\n  "/example2/recording1.vrs": "2",\n}\n\n'})})]})}function m(e={}){const{wrapper:t}={...(0,r.R)(),...e.components};return t?(0,n.jsx)(t,{...e,children:(0,n.jsx)(d,{...e})}):d(e)}},28453:(e,t,s)=>{s.d(t,{R:()=>o,x:()=>i});var a=s(96540);const n={},r=a.createContext(n);function o(e){const t=a.useContext(r);return a.useMemo((function(){return"function"==typeof e?e(t):{...t,...e}}),[t,e])}function i(e){let t;return t=e.disableParentContext?"function"==typeof e.components?e.components(n):e.components||n:o(e.components),a.createElement(r.Provider,{value:t},e.children)}}}]);