"use strict";(self.webpackChunkwebsite=self.webpackChunkwebsite||[]).push([[840],{63993:(e,a,r)=>{r.r(a),r.d(a,{assets:()=>c,contentTitle:()=>o,default:()=>p,frontMatter:()=>l,metadata:()=>d,toc:()=>u});var t=r(74848),n=r(28453),i=r(49489),s=r(7227);const l={sidebar_position:10,title:"Data Provider"},o="Aria Data Provider Code Snippets",d={id:"data_utilities/core_code_snippets/data_provider",title:"Data Provider",description:"In this section, we introduce the Python/C++ API to access sensor data in Project Aria VRS files (projectariatools/main/core/dataprovider).",source:"@site/docs/data_utilities/core_code_snippets/data_provider.mdx",sourceDirName:"data_utilities/core_code_snippets",slug:"/data_utilities/core_code_snippets/data_provider",permalink:"/projectaria_tools/docs/data_utilities/core_code_snippets/data_provider",draft:!1,unlisted:!1,editUrl:"https://github.com/facebookresearch/projectaria_tools/tree/main/website/docs/data_utilities/core_code_snippets/data_provider.mdx",tags:[],version:"current",sidebarPosition:10,frontMatter:{sidebar_position:10,title:"Data Provider"},sidebar:"tutorialSidebar",previous:{title:"C++ Visualization",permalink:"/projectaria_tools/docs/data_utilities/visualization/visualization_cpp"},next:{title:"Image",permalink:"/projectaria_tools/docs/data_utilities/core_code_snippets/image"}},c={},u=[{value:"Open a VRS file",id:"open-a-vrs-file",level:3},{value:"Mapping between labels and stream ids",id:"mapping-between-labels-and-stream-ids",level:3},{value:"Random access data by index",id:"random-access-data-by-index",level:3},{value:"Random access data by timestamp",id:"random-access-data-by-timestamp",level:3},{value:"Deliver all sensor data in VRS",id:"deliver-all-sensor-data-in-vrs",level:3}];function m(e){const a={a:"a",code:"code",h1:"h1",h3:"h3",header:"header",li:"li",p:"p",pre:"pre",ul:"ul",...(0,n.R)(),...e.components};return(0,t.jsxs)(t.Fragment,{children:[(0,t.jsx)(a.header,{children:(0,t.jsx)(a.h1,{id:"aria-data-provider-code-snippets",children:"Aria Data Provider Code Snippets"})}),"\n",(0,t.jsxs)(a.p,{children:["In this section, we introduce the Python/C++ API to access sensor data in Project Aria VRS files (",(0,t.jsx)(a.a,{href:"https://github.com/facebookresearch/projectaria_tools/blob/main/core/data_provider",children:"projectaria_tools/main/core/data_provider"}),")."]}),"\n",(0,t.jsx)(a.h3,{id:"open-a-vrs-file",children:"Open a VRS file"}),"\n",(0,t.jsxs)(i.default,{groupId:"programming-language",children:[(0,t.jsx)(s.default,{value:"python",label:"Python",children:(0,t.jsx)(a.pre,{children:(0,t.jsx)(a.code,{className:"language-python",children:'from projectaria_tools.core import data_provider\nfrom projectaria_tools.core.sensor_data import TimeDomain, TimeQueryOptions\nfrom projectaria_tools.core.stream_id import RecordableTypeId, StreamId\n\nvrsfile = "example.vrs"\nprovider = data_provider.create_vrs_data_provider(vrsfile)\nassert provider is not None, "Cannot open file"\n'})})}),(0,t.jsx)(s.default,{value:"cpp",label:"C++",children:(0,t.jsx)(a.pre,{children:(0,t.jsx)(a.code,{className:"language-cpp",children:'#include <dataprovider/VrsDataProvider.h>\nusing namespace projectaria::tools::data_provider;\n\nstd::string vrsfile = "example.vrs"\nauto maybeProvider = createVrsDataProvider(vrsFilename);\nXR_CHECK(maybeProvider, "Cannot open file");\nVrsDataProvider& provider = *maybeProvider;\n'})})})]}),"\n",(0,t.jsx)(a.h3,{id:"mapping-between-labels-and-stream-ids",children:"Mapping between labels and stream ids"}),"\n",(0,t.jsxs)(i.default,{groupId:"programming-language",children:[(0,t.jsxs)(s.default,{value:"python",label:"Python",children:[(0,t.jsxs)(a.p,{children:["Stream IDs can be mapped from labels by using ",(0,t.jsx)(a.code,{children:"get_stream_id_from_label"}),":"]}),(0,t.jsx)(a.pre,{children:(0,t.jsx)(a.code,{className:"language-python",children:'stream_id = provider.get_stream_id_from_label("camera-slam-left")\n'})}),(0,t.jsxs)(a.p,{children:["Inversely, you can retrieve a label from a stream ID by using ",(0,t.jsx)(a.code,{children:"get_stream_id_from_label"}),":"]}),(0,t.jsx)(a.pre,{children:(0,t.jsx)(a.code,{className:"language-python",children:'label = provider.get_label_from_stream_id(StreamId("1201-1"))\n'})})]}),(0,t.jsxs)(s.default,{value:"cpp",label:"C++",children:[(0,t.jsxs)(a.p,{children:["Stream IDs can be mapped from labels by using ",(0,t.jsx)(a.code,{children:"getStreamIdFromLabel"}),":"]}),(0,t.jsx)(a.pre,{children:(0,t.jsx)(a.code,{className:"language-cpp",children:'auto streamId = provider.getStreamIdFromLabel("camera-slam-left");\n'})}),(0,t.jsxs)(a.p,{children:["Inversely, you can retrieve a label from a stream id by using ",(0,t.jsx)(a.code,{children:"getLabelFromStreamId"}),"."]}),(0,t.jsx)(a.pre,{children:(0,t.jsx)(a.code,{className:"language-cpp",children:'auto label = provider.getLabelFromStreamId(StreamId::fromNumericalName("1201-1"));\n'})})]})]}),"\n",(0,t.jsx)(a.h3,{id:"random-access-data-by-index",children:"Random access data by index"}),"\n",(0,t.jsxs)(i.default,{groupId:"programming-language",children:[(0,t.jsx)(s.default,{value:"python",label:"Python",children:(0,t.jsx)(a.pre,{children:(0,t.jsx)(a.code,{className:"language-python",children:"for stream_id in provider.get_all_streams():\n  for i in range(0, provider.get_num_data(stream_id)):\n    sensor_data =  provider.get_sensor_data_by_index(stream_id, i)\n"})})}),(0,t.jsx)(s.default,{value:"cpp",label:"C++",children:(0,t.jsx)(a.pre,{children:(0,t.jsx)(a.code,{className:"language-cpp",children:"for (const auto& streamId : provider.getAllStreams()) {\n  for( size_t i =  0 ; i < provider.getNumData(streamId); ++i) {\n    auto sensorData =  provider.getSensorDataByIndex(streamId, i);\n  }\n}\n"})})})]}),"\n",(0,t.jsx)(a.h3,{id:"random-access-data-by-timestamp",children:"Random access data by timestamp"}),"\n",(0,t.jsxs)(a.p,{children:["Project Aria data has four kinds of TimeDomain entries. We strongly recommend always working with ",(0,t.jsx)(a.code,{children:"DEVICE_TIME"})," when using single-device Aria data. The ",(0,t.jsx)(a.code,{children:"TIME_CODE"})," TimeDomain is used when synchronizing time across multiple devices. Go to ",(0,t.jsx)(a.a,{href:"/projectaria_tools/docs/data_formats/aria_vrs/timestamps_in_aria_vrs",children:"Timestamps in Aria VRS Files"})," for more information."]}),"\n",(0,t.jsxs)(i.default,{groupId:"programming-language",children:[(0,t.jsxs)(s.default,{value:"python",label:"Python",children:[(0,t.jsxs)(a.ul,{children:["\n",(0,t.jsx)(a.li,{children:"TimeDomain.RECORD_TIME"}),"\n",(0,t.jsx)(a.li,{children:"TimeDomain.DEVICE_TIME - recommended"}),"\n",(0,t.jsx)(a.li,{children:"TimeDomain.HOST_TIME"}),"\n",(0,t.jsx)(a.li,{children:"TimeDomain.TIME_CODE - for multiple devices"}),"\n"]}),(0,t.jsx)(a.p,{children:"You can also search using three different time query options:"}),(0,t.jsxs)(a.ul,{children:["\n",(0,t.jsxs)(a.li,{children:["TimeQueryOptions.BEFORE (default): last data with ",(0,t.jsx)(a.code,{children:"t <= t_query"})]}),"\n",(0,t.jsxs)(a.li,{children:["TimeQueryOptions.AFTER : first data with ",(0,t.jsx)(a.code,{children:"t >= t_query"})]}),"\n",(0,t.jsxs)(a.li,{children:["TimeQueryOptions.CLOSEST : the data where ",(0,t.jsx)(a.code,{children:"|t - t_query|"})," is smallest"]}),"\n"]}),(0,t.jsx)(a.pre,{children:(0,t.jsx)(a.code,{className:"language-python",children:"for stream_id in provider.get_all_streams():\n  t_first = provider.get_first_time_ns(stream_id, TimeDomain.DEVICE_TIME)\n  t_last = provider.get_last_time_ns(stream_id, TimeDomain.DEVICE_TIME)\n  query_timestamp = (t_first + t_last) // 2 # example query timestamp\n  sensor_data = provider.get_sensor_data_by_time_ns(stream_id, query_timestamp, TimeDomain.DEVICE_TIME, TimeQueryOptions.CLOSEST)\n"})})]}),(0,t.jsxs)(s.default,{value:"cpp",label:"C++",children:[(0,t.jsxs)(a.ul,{children:["\n",(0,t.jsx)(a.li,{children:"TimeDomain::RecordTime"}),"\n",(0,t.jsx)(a.li,{children:"TimeDomain::DeviceTime - recommended"}),"\n",(0,t.jsx)(a.li,{children:"TimeDomain::HostTime"}),"\n",(0,t.jsx)(a.li,{children:"TimeDomain::TimeCode - for multiple devices"}),"\n"]}),(0,t.jsx)(a.p,{children:"You can also search using three different time query options:"}),(0,t.jsxs)(a.ul,{children:["\n",(0,t.jsxs)(a.li,{children:["TimeQueryOptions::Before : last data with ",(0,t.jsx)(a.code,{children:"t <= t_query"})]}),"\n",(0,t.jsxs)(a.li,{children:["TimeQueryOptions::After : first data with ",(0,t.jsx)(a.code,{children:"t >= t_query"})]}),"\n",(0,t.jsxs)(a.li,{children:["TimeQueryOptions::Closest : the data where ",(0,t.jsx)(a.code,{children:"|t - t_query|"})," is smallest"]}),"\n"]}),(0,t.jsx)(a.pre,{children:(0,t.jsx)(a.code,{className:"language-cpp",children:"for (const auto& streamId : provider.getAllStreams()) {\n  int64_t tFirst = provider.getFirstTimeNs(streamId, TimeDomain::DeviceTime);\n  int64_t tLast = provider.getLastTimeNs(streamId, TimeDomain::DeviceTime);\n\n  auto queryTimestamp = (tFirst + tLast) / 2; // example query timestamp\n  auto sensorData = provider.getSensorDataByTimeNs(streamId, queryTimestamp, TimeDomain::DeviceTime, TimeQueryOptions::Closest);\n}\n"})})]})]}),"\n",(0,t.jsx)(a.h3,{id:"deliver-all-sensor-data-in-vrs",children:"Deliver all sensor data in VRS"}),"\n",(0,t.jsxs)(i.default,{groupId:"programming-language",children:[(0,t.jsxs)(s.default,{value:"python",label:"Python",children:[(0,t.jsx)(a.p,{children:"Async iterator to deliver sensor data for all streams in device time order:"}),(0,t.jsx)(a.pre,{children:(0,t.jsx)(a.code,{className:"language-python",children:"for data in provider.deliver_queued_sensor_data():\n  print(data.get_time_ns(TimeDomain.DEVICE_TIME))\n"})}),(0,t.jsx)(a.p,{children:"Alternatively, you can use iterator-type syntax:"}),(0,t.jsx)(a.pre,{children:(0,t.jsx)(a.code,{className:"language-python",children:"seq = provider.deliver_queued_sensor_data()\nobj = next(seq)\nwhile True:\n  print(obj.get_time_ns(TimeDomain.DEVICE_TIME))\n  try:\n    obj = next(seq)\n  except StopIteration:\n    break\n"})}),(0,t.jsx)(a.p,{children:"Deliver with sub-stream selection, time truncation, and frame rate sub-sampling:"}),(0,t.jsx)(a.pre,{children:(0,t.jsx)(a.code,{className:"language-python",children:'# Starts by default options which activates all sensors\ndeliver_option = provider.get_default_deliver_queued_options()\n\n# Only play data from two cameras, also reduce framerate to half of vrs\ndeliver_option.deactivate_stream_all()\nfor label in ["camera-slam-left", "camera-slam-right"]:\n  streamId = provider.get_stream_id_from_label(label)\n  deliver_option.activate_stream(streamId)\n  deliver_option.set_subsample_rate(streamId, 2)\n\n# skip first 100ns\ndeliver_option.set_truncate_first_device_time_ns(100)\nfor data in provider.deliver_queued_sensor_data() :\n  print(data.get_time_ns(TimeDomain.DEVICE_TIME))\n'})})]}),(0,t.jsxs)(s.default,{value:"cpp",label:"C++",children:[(0,t.jsx)(a.p,{children:"Async iterator to deliver sensor data for all streams in device time order:"}),(0,t.jsx)(a.pre,{children:(0,t.jsx)(a.code,{className:"language-cpp",children:"for (const SensorData& data : provider.deliverQueuedSensorData()) {\n  std::cout << data.getTimeNs(TimeDomain::DeviceTime) << std::endl;\n}\n"})}),(0,t.jsx)(a.p,{children:"Alternatively, you can use iterator-type syntax:"}),(0,t.jsx)(a.pre,{children:(0,t.jsx)(a.code,{className:"language-cpp",children:"auto seq = provider.deliverQueuedSensorData();\nfor (const auto& it = seq.begin(), it != seq.end(); ++it) {\n  SensorData data = *it;\n  std::cout << data.getTimeNs(TimeDomain::DeviceTime) << std::endl;\n}\n"})}),(0,t.jsx)(a.p,{children:"Deliver with sub-stream selection, time truncation, and frame rate sub-sampling:"}),(0,t.jsx)(a.pre,{children:(0,t.jsx)(a.code,{className:"language-cpp",children:'// Starts by default options which activates all sensors\ndeliverOption = provider.getDefaultDeliverQueuedOptions();\ndeliverOption.deactivateStreamAll();\n\n// Only play data from two cameras, also reduce framerate to half of vrs\nfor (const auto& label : {"camera-slam-left", "camera-slam-right"}) {\n  std::optional<vrs::StreamId> maybeStreamId = provider.getStreamIdFromLabel(label);\n  if (maybeStreamId) {\n    deliverOption.activateStream(maybeStreamId.value());\n    deliverOption.setSubsampleRate(maybeStreamId.value(), 2);\n  }\n}\n\n// skip first 100ns\ndeliverOption.setTruncateFirstDeviceTimeNs(100);\nfor (const SensorData& data : provider.deliverQueuedSensorData(deliverOption)) {\n  std::cout << data.getTimeNs(TimeDomain::DeviceTime) << std::endl;\n}\n'})})]})]})]})}function p(e={}){const{wrapper:a}={...(0,n.R)(),...e.components};return a?(0,t.jsx)(a,{...e,children:(0,t.jsx)(m,{...e})}):m(e)}},7227:(e,a,r)=>{r.r(a),r.d(a,{default:()=>s});r(96540);var t=r(34164);const n={tabItem:"tabItem_Ymn6"};var i=r(74848);function s(e){let{children:a,hidden:r,className:s}=e;return(0,i.jsx)("div",{role:"tabpanel",className:(0,t.A)(n.tabItem,s),hidden:r,children:a})}},49489:(e,a,r)=>{r.r(a),r.d(a,{default:()=>y});var t=r(96540),n=r(34164),i=r(24245),s=r(56347),l=r(36494),o=r(62814),d=r(45167),c=r(69900);function u(e){return t.Children.toArray(e).filter((e=>"\n"!==e)).map((e=>{if(!e||(0,t.isValidElement)(e)&&function(e){const{props:a}=e;return!!a&&"object"==typeof a&&"value"in a}(e))return e;throw new Error(`Docusaurus error: Bad <Tabs> child <${"string"==typeof e.type?e.type:e.type.name}>: all children of the <Tabs> component should be <TabItem>, and every <TabItem> should have a unique "value" prop.`)}))?.filter(Boolean)??[]}function m(e){const{values:a,children:r}=e;return(0,t.useMemo)((()=>{const e=a??function(e){return u(e).map((e=>{let{props:{value:a,label:r,attributes:t,default:n}}=e;return{value:a,label:r,attributes:t,default:n}}))}(r);return function(e){const a=(0,d.XI)(e,((e,a)=>e.value===a.value));if(a.length>0)throw new Error(`Docusaurus error: Duplicate values "${a.map((e=>e.value)).join(", ")}" found in <Tabs>. Every value needs to be unique.`)}(e),e}),[a,r])}function p(e){let{value:a,tabValues:r}=e;return r.some((e=>e.value===a))}function h(e){let{queryString:a=!1,groupId:r}=e;const n=(0,s.W6)(),i=function(e){let{queryString:a=!1,groupId:r}=e;if("string"==typeof a)return a;if(!1===a)return null;if(!0===a&&!r)throw new Error('Docusaurus error: The <Tabs> component groupId prop is required if queryString=true, because this value is used as the search param name. You can also provide an explicit value such as queryString="my-search-param".');return r??null}({queryString:a,groupId:r});return[(0,o.aZ)(i),(0,t.useCallback)((e=>{if(!i)return;const a=new URLSearchParams(n.location.search);a.set(i,e),n.replace({...n.location,search:a.toString()})}),[i,n])]}function v(e){const{defaultValue:a,queryString:r=!1,groupId:n}=e,i=m(e),[s,o]=(0,t.useState)((()=>function(e){let{defaultValue:a,tabValues:r}=e;if(0===r.length)throw new Error("Docusaurus error: the <Tabs> component requires at least one <TabItem> children component");if(a){if(!p({value:a,tabValues:r}))throw new Error(`Docusaurus error: The <Tabs> has a defaultValue "${a}" but none of its children has the corresponding value. Available values are: ${r.map((e=>e.value)).join(", ")}. If you intend to show no default tab, use defaultValue={null} instead.`);return a}const t=r.find((e=>e.default))??r[0];if(!t)throw new Error("Unexpected error: 0 tabValues");return t.value}({defaultValue:a,tabValues:i}))),[d,u]=h({queryString:r,groupId:n}),[v,_]=function(e){let{groupId:a}=e;const r=function(e){return e?`docusaurus.tab.${e}`:null}(a),[n,i]=(0,c.Dv)(r);return[n,(0,t.useCallback)((e=>{r&&i.set(e)}),[r,i])]}({groupId:n}),f=(()=>{const e=d??v;return p({value:e,tabValues:i})?e:null})();(0,l.A)((()=>{f&&o(f)}),[f]);return{selectedValue:s,selectValue:(0,t.useCallback)((e=>{if(!p({value:e,tabValues:i}))throw new Error(`Can't select invalid tab value=${e}`);o(e),u(e),_(e)}),[u,_,i]),tabValues:i}}var _=r(11062);const f={tabList:"tabList__CuJ",tabItem:"tabItem_LNqP"};var b=r(74848);function g(e){let{className:a,block:r,selectedValue:t,selectValue:s,tabValues:l}=e;const o=[],{blockElementScrollPositionUntilNextRender:d}=(0,i.a_)(),c=e=>{const a=e.currentTarget,r=o.indexOf(a),n=l[r].value;n!==t&&(d(a),s(n))},u=e=>{let a=null;switch(e.key){case"Enter":c(e);break;case"ArrowRight":{const r=o.indexOf(e.currentTarget)+1;a=o[r]??o[0];break}case"ArrowLeft":{const r=o.indexOf(e.currentTarget)-1;a=o[r]??o[o.length-1];break}}a?.focus()};return(0,b.jsx)("ul",{role:"tablist","aria-orientation":"horizontal",className:(0,n.A)("tabs",{"tabs--block":r},a),children:l.map((e=>{let{value:a,label:r,attributes:i}=e;return(0,b.jsx)("li",{role:"tab",tabIndex:t===a?0:-1,"aria-selected":t===a,ref:e=>o.push(e),onKeyDown:u,onClick:c,...i,className:(0,n.A)("tabs__item",f.tabItem,i?.className,{"tabs__item--active":t===a}),children:r??a},a)}))})}function x(e){let{lazy:a,children:r,selectedValue:i}=e;const s=(Array.isArray(r)?r:[r]).filter(Boolean);if(a){const e=s.find((e=>e.props.value===i));return e?(0,t.cloneElement)(e,{className:(0,n.A)("margin-top--md",e.props.className)}):null}return(0,b.jsx)("div",{className:"margin-top--md",children:s.map(((e,a)=>(0,t.cloneElement)(e,{key:a,hidden:e.props.value!==i})))})}function j(e){const a=v(e);return(0,b.jsxs)("div",{className:(0,n.A)("tabs-container",f.tabList),children:[(0,b.jsx)(g,{...a,...e}),(0,b.jsx)(x,{...a,...e})]})}function y(e){const a=(0,_.default)();return(0,b.jsx)(j,{...e,children:u(e.children)},String(a))}},28453:(e,a,r)=>{r.d(a,{R:()=>s,x:()=>l});var t=r(96540);const n={},i=t.createContext(n);function s(e){const a=t.useContext(i);return t.useMemo((function(){return"function"==typeof e?e(a):{...a,...e}}),[a,e])}function l(e){let a;return a=e.disableParentContext?"function"==typeof e.components?e.components(n):e.components||n:s(e.components),t.createElement(i.Provider,{value:a},e.children)}}}]);