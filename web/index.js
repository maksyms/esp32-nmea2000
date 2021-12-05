let self = this;
let lastUpdate = (new Date()).getTime();
let reloadConfig = false;
function addEl(type, clazz, parent, text) {
    let el = document.createElement(type);
    if (clazz) {
        if (!(clazz instanceof Array)) {
            clazz = clazz.split(/  */);
        }
        clazz.forEach(function (ce) {
            el.classList.add(ce);
        });
    }
    if (text) el.textContent = text;
    if (parent) parent.appendChild(el);
    return el;
}
function forEl(query, callback,base) {
    if (! base) base=document;
    let all = base.querySelectorAll(query);
    for (let i = 0; i < all.length; i++) {
        callback(all[i]);
    }
}
function closestParent(element,clazz){
    while (true){
        let parent=element.parentElement;
        if (! parent) return;
        if (parent.classList.contains(clazz)) return parent;
        element=parent;
    }
}
function alertRestart() {
    reloadConfig = true;
    alert("Board reset triggered, reconnect WLAN if necessary");
}
function getJson(url) {
    return fetch(url)
        .then(function (r) { return r.json() });
}
function getText(url){
    return fetch(url)
        .then(function (r) { return r.text() });
}
function reset() {
    fetch('/api/reset');
    alertRestart();
}
function update() {
    let now = (new Date()).getTime();
    let ce = document.getElementById('connected');
    if (ce) {
        if ((lastUpdate + 3000) > now) {
            ce.classList.add('ok');
        }
        else {
            ce.classList.remove('ok');
        }
    }
    getJson('/api/status')
        .then(function (jsonData) {
            for (let k in jsonData) {
                if (typeof (jsonData[k]) === 'object') {
                    for (let sk in jsonData[k]) {
                        let key = k + "." + sk;
                        if (typeof (jsonData[k][sk]) === 'object') {
                            //msg details
                            updateMsgDetails(key, jsonData[k][sk]);
                        }
                        else {
                            let el = document.getElementById(key);
                            if (el) el.textContent = jsonData[k][sk];
                        }
                    }
                }
                else {
                    let el = document.getElementById(k);
                    if (el) el.textContent = jsonData[k];
                }
            }
            lastUpdate = (new Date()).getTime();
            if (reloadConfig) {
                reloadConfig = false;
                resetForm();
            }
        })
}
function resetForm(ev) {
    getJson("/api/config")
        .then(function (jsonData) {
            for (let k in jsonData) {
                let el = document.querySelector("[name='" + k + "']");
                if (el) {
                    let v = jsonData[k];
                    el.value = v;
                    el.setAttribute('data-loaded', v);
                    let changeEvent = new Event('change');
                    el.dispatchEvent(changeEvent);
                }
            }
            let name=jsonData.systemName;
            if (name){
                let el=document.getElementById('headline');
                if (el) el.textContent=name;
                document.title=name;
            }
        });
}
function checkMinMax(v,allValues,def){
    let parsed=parseFloat(v);
    if (isNaN(parsed)) return "must be a number";
    if (def.min !== undefined){
        if (parsed < parseFloat(def.min)) return "must be >= "+def.min;
    }
    if (def.max !== undefined){
        if (parsed > parseFloat(def.max)) return "must be <= "+def.max;
    }
}

function checkSystemName(v) {
    //2...32 characters for ssid
    let allowed = v.replace(/[^a-zA-Z0-9]*/g, '');
    if (allowed != v) return "contains invalid characters, only a-z, A-Z, 0-9";
    if (v.length < 2 || v.length > 32) return "invalid length (2...32)";
}
function checkApPass(v) {
    //min 8 characters
    if (v.length < 8) {
        return "password must be at least 8 characters";
    }
}

function checkXDR(v,allValues){
    if (! v) return;
    let parts=v.split(',');
    if (parseInt(parts[1])  == 0) return;
    if (parseInt(parts[1]) != 0 && ! parts[6]) return "missing transducer name";
    for (let k in allValues){
        if (! k.match(/^XDR/)) continue;
        let cmp=allValues[k];
        if (cmp == v){
            return "same mapping already defined in "+k;
        }
        let cmpParts=cmp.split(',');
        if (parseInt(cmpParts[1]) != 0 && parts[6] == cmpParts[6]){
            return "transducer "+parts[6]+" already defined in "+k;
        }
        //check similar mappings
        if (parts[0]==cmpParts[0] && parts[2] == cmpParts[2] && parts[3] == cmpParts[3]){
            if (parts[4] == cmpParts[4] && parts[5] == cmpParts[5]){
                return "mapping for the same entity already defined in "+k;
            }
            if ((parseInt(parts[4]) == 0 && parseInt(cmpParts[4]) == 1)||
            (parseInt(parts[4]) == 1 && parseInt(cmpParts[4]) == 0)
            ){
                //ignore and single for the same mapping
                return "mapping for the same entity already defined in "+k;
            }
        }
    }
}
function changeConfig() {
    let url = "/api/setConfig?";
    let values = document.querySelectorAll('.configForm select , .configForm input');
    let allValues={};
    for (let i = 0; i < values.length; i++) {
        let v = values[i];
        let name = v.getAttribute('name');
        if (!name) continue;
        if (name.indexOf("_") >= 0) continue;
        let check = v.getAttribute('data-check');
        if (check) {
            if (typeof (self[check]) === 'function') {
                let res = self[check](v.value,allValues,getConfigDefition(name));
                if (res) {
                    let value = v.value;
                    if (v.type === 'password') value = "******";
                    alert("invalid config for " + v.getAttribute('name') + "(" + value + "):\n" + res);
                    return;
                }
            }
        }
        allValues[name]=v.value;
        url += name + "=" + encodeURIComponent(v.value) + "&";
    }
    getJson(url)
        .then(function (status) {
            if (status.status == 'OK') {
                alertRestart();
            }
            else {
                alert("unable to set config: " + status.status);
            }
        })
}
function factoryReset() {
    if (!confirm("Really delete all configuration?\n" +
        "This will reset all your Wifi settings and disconnect you.")) {
        return;
    }
    getJson("/api/resetConfig")
        .then(function (status) {
            alertRestart();
        })
}
function createCounterDisplay(parent,label,key,isEven){
    let clazz="row icon-row counter-row";
    if (isEven) clazz+=" even";
    let row=addEl('div',clazz,parent);
    let icon=addEl('span','icon icon-more',row);
    addEl('span','label',row,label);
    let value=addEl('span','value',row,'---');
    value.setAttribute('id',key+".sumOk");
    let display=addEl('div',clazz+" msgDetails hidden",parent);
    display.setAttribute('id',key+".ok");
    row.addEventListener('click',function(ev){
        let rs=display.classList.toggle('hidden');
        if (rs){
            icon.classList.add('icon-more');
            icon.classList.remove('icon-less');
        }
        else{
            icon.classList.remove('icon-more');
            icon.classList.add('icon-less');
        }
    });
}

function updateMsgDetails(key, details) {
    forEl('.msgDetails', function (frame) {
        if (frame.getAttribute('id') !== key) return;
        for (let k in details) {
            let el = frame.querySelector("[data-id=\"" + k + "\"] ");
            if (!el) {
                el = addEl('div', 'row', frame);
                let cv = addEl('span', 'label', el, k);
                cv = addEl('span', 'value', el, details[k]);
                cv.setAttribute('data-id', k);
            }
            else {
                el.textContent = details[k];
            }
        }
        forEl('.value',function(el){
            let k=el.getAttribute('data-id');
            if (k && ! details[k]){
                el.parentElement.remove();
            }
        },frame);
    });
}
let counters={
    count2Kin: 'NMEA2000 in',
    count2Kout: 'NMEA2000 out',
    countTCPin: 'TCP in',
    countTCPout: 'TCP out',
    countUSBin: 'USB in',
    countUSBout: 'USB out',
    countSerialIn: 'Serial in',
    countSerialOut: 'Serial out'
}
function showOverlay(text, isHtml) {
    let el = document.getElementById('overlayContent');
    if (isHtml) {
        el.innerHTML = text;
        el.classList.remove("text");
    }
    else {
        el.textContent = text;
        el.classList.add("text");
    }
    let container = document.getElementById('overlayContainer');
    container.classList.remove('hidden');
}
function hideOverlay() {
    let container = document.getElementById('overlayContainer');
    container.classList.add('hidden');
}
function checkChange(el, row,name) {
    let loaded = el.getAttribute('data-loaded');
    if (loaded !== undefined) {
        if (loaded != el.value) {
            row.classList.add('changed');
        }
        else {
            row.classList.remove("changed");
        }
    }
    let dependend=conditionRelations[name];
    if (dependend){
        for (let el in dependend){
            checkCondition(dependend[el]);
        }
    }
}
let configDefinitions={};
let xdrConfig={};
//a map between the name of a config item and a list of dependend items
let conditionRelations={};
function getConfigDefition(name){
    if (! name) return {};
    let def;
    for (let k in configDefinitions){
        if (configDefinitions[k].name === name){
            def=configDefinitions[k];
            break;
        }
    }
    if (! def) return {};
    return def;
}
function getConditions(name){
    let def=getConfigDefition(name);
    if (! def) return;
    let condition=def.condition;
    if (! condition) return;
    if (! (condition instanceof Array)) condition=[condition];
    return condition;
}
function checkCondition(element){
    let name=element.getAttribute('name');
    let condition=getConditions(name);
    if (! condition) return;
    let visible=false;
    condition.forEach(function(cel){
        let lvis=true;
        for (let k in cel){
            let item=document.querySelector('[name='+k+']');
            if (item){
                if (item.value != cel[k]) lvis=false;
            }
        }
        if (lvis) visible=true;
    });
    let row=closestParent(element,'row');
    if (!row) return;
    if (visible) row.classList.remove('hidden');
    else row.classList.add('hidden');
}
function createInput(configItem, frame,clazz) {
    let el;
    if (configItem.type === 'boolean' || configItem.type === 'list' || configItem.type == 'boatData') {
        el=addEl('select',clazz,frame);
        el.setAttribute('name', configItem.name)
        let slist = [];
        if (configItem.list) {
            configItem.list.forEach(function (v) {
                if (v instanceof Object){
                    slist.push({l:v.l,v:v.v});
                }
                else{
                    slist.push({ l: v, v: v });
                }
            })
        }
        else if (configItem.type != 'boatData') {
            slist.push({ l: 'on', v: 'true' })
            slist.push({ l: 'off', v: 'false' })
        }
        slist.forEach(function (sitem) {
            let sitemEl = addEl('option','',el,sitem.l);
            sitemEl.setAttribute('value', sitem.v);
        })
        if (configItem.type == 'boatData'){
            el.classList.add('boatDataSelect');
        }
        return el;
    }
    if (configItem.type === 'filter') {
        return createFilterInput(configItem,frame,clazz);    
    }
    if (configItem.type === 'xdr'){
        return createXdrInput(configItem,frame,clazz);
    }
    el = addEl('input',clazz,frame);
    el.setAttribute('name', configItem.name)
    if (configItem.type === 'password') {
        el.setAttribute('type', 'password');
        let vis = addEl('span', 'icon-eye icon', frame);
        vis.addEventListener('click', function (v) {
            if (vis.classList.toggle('active')) {
                el.setAttribute('type', 'text');
            }
            else {
                el.setAttribute('type', 'password');
            }
        });
    }
    else if (configItem.type === 'number') {
        el.setAttribute('type', 'number');
    }
    else {
        el.setAttribute('type', 'text');
    }
    return el;
}

function updateSelectList(item,slist){
    item.innerHTML='';
    let idx=0;
    slist.forEach(function (sitem) {
        let sitemEl = addEl('option','',item,sitem.l);
        sitemEl.setAttribute('value', sitem.v !== undefined?sitem.v:idx);
        idx++;
    })
}
function getXdrCategories(){
    let rt=[];
    for (let c in xdrConfig){
        if (xdrConfig[c].enabled !== false){
            rt.push({l:c,v:xdrConfig[c].id});
        }
    }
    return rt;
}
function getXdrCategoryName(cid){
    category=parseInt(cid);
    for (let c in xdrConfig){
        let base=xdrConfig[c];
        if (parseInt(base.id) == category){
            return c;
        }
    }
}
function getXdrCategory(cid){
    category=parseInt(cid);
    for (let c in xdrConfig){
        let base=xdrConfig[c];
        if (parseInt(base.id) == category){
            return base;
        }
    }
    return {};
}
function getXdrSelectors(category){
    let base=getXdrCategory(category);
    return base.selector || [];
}
function getXdrFields(category){
    let base=getXdrCategory(category);
    if (!base.fields) return [];
    let rt=[];
    base.fields.forEach(function(f){
        if (f.t === undefined) return;
        if (parseInt(f.t) == 99) return; //unknown type
        rt.push(f);
    });
    return rt;
}

function createXdrLine(parent,label){
    let d=addEl('div','xdrline',parent);
    addEl('span','xdrlabel',d,label);
    return d;
}
function showHideXdr(el,show,useParent){
    if (useParent) el=el.parentElement;
    if (show) el.classList.remove('xdrunused');
    else el.classList.add('xdrunused');
}

function createXdrInput(configItem,frame){
    let configCategory=configItem.category;
    let el = addEl('div','xdrinput',frame);
    let d=createXdrLine(el,'Direction');
    let direction=createInput({
        type:'list',
        name: configItem.name+"_dir",
        list: [
            //GwXDRMappingDef::Direction
            {l:'off',v:0},
            {l:'bidir',v:1},
            {l:'to2K',v:2},
            {l:'from2K',v:3}
        ]
    },d,'xdrdir');
    d=createXdrLine(el,'Category');
    let category=createInput({
        type: 'list',
        name: configItem.name+"_cat",
        list:getXdrCategories()
    },d,'xdrcat');
    d=createXdrLine(el,'Source');
    let selector=createInput({
        type: 'list',
        name: configItem.name+"_sel",
        list:[]
    },d,'xdrsel');
    d=createXdrLine(el,'Field');
    let field=createInput({
        type:'list',
        name: configItem.name+'_field',
        list: []
    },d,'xdrfield');
    d=createXdrLine(el,'Instance');
    let imode=createInput({
        type:'list',
        name: configItem.name+"_imode",
        list:[
            //GwXDRMappingDef::InstanceMode
            {l:'single',v:0},
            {l:'ignore',v:1},
            {l:'auto',v:2}
        ]
    },d,'xdrimode');
    let instance=createInput({
        type:'number',
        name: configItem.name+"_instance",
    },d,'xdrinstance');
    d=createXdrLine(el,'Transducer');
    let xdrName=createInput({
        type:'text',
        name: configItem.name+"_xdr"
    },d,'xdrname');
    d=createXdrLine(el,'Example');
    let example=addEl('div','xdrexample',d,'');
    let data = addEl('input','xdrvalue',el);
    data.setAttribute('type', 'hidden');
    data.setAttribute('name', configItem.name);
    let changeFunction = function () {
        let parts=data.value.split(',');
        direction.value=parts[1] || 0;
        category.value=parts[0] || 0;
        let selectors=getXdrSelectors(category.value);
        updateSelectList(selector,selectors);
        showHideXdr(selector,selectors.length>0);
        let fields=getXdrFields(category.value);
        updateSelectList(field,fields);
        showHideXdr(field,fields.length>0);
        selector.value=parts[2]||0;
        field.value=parts[3]||0;
        imode.value=parts[4]||0;
        instance.value=parts[5]||0;
        showHideXdr(instance,imode.value == 0);
        xdrName.value=parts[6]||'';
        let used=isXdrUsed(data);
        let modified=data.value != data.getAttribute('data-loaded');
        forEl('[data-category='+configCategory+']',function(el){
            if (used) {
                el.classList.add('xdrcused');
                el.classList.remove('xdrcunused');
                forEl('.categoryAdd',function(add){
                    add.textContent=xdrName.value;
                },el);
            }
            else {
                el.classList.remove('xdrcused');
                el.classList.add('xdrcunused');
                forEl('.categoryAdd',function(add){
                    add.textContent='';
                },el);
            }
            if (modified){
                el.classList.add('changed');
            }
            else{
                el.classList.remove('changed');
            }
        });
        if (used){
            getText('/api/xdrExample?mapping='+encodeURIComponent(data.value)+'&value=2.1')
                .then(function(txt){
                    example.textContent=txt;
                })
        }
        else{
            example.textContent='';
        }
    }
    let updateFunction = function (evt) {
        if (evt.target == category){
            selector.value=0;
            field.value=0;
            instance.value=0;
        }
        let txt=category.value+","+direction.value+","+
            selector.value+","+field.value+","+imode.value;
        let instanceValue=parseInt(instance.value||0);
        if (isNaN(instanceValue)) instanceValue=0;
        if (instanceValue<0) instanceValue=0;
        if (instanceValue>255) instanceValue=255;
        txt+=","+instanceValue;
        let xdr=xdrName.value.replace(/[^a-zA-Z0-9]/g,'');
        xdr=xdr.substr(0,12);
        txt+=","+xdr;    
        data.value=txt;
        let ev=new Event('change');
        data.dispatchEvent(ev);
    }
    category.addEventListener('change',updateFunction);
    direction.addEventListener('change',updateFunction);
    selector.addEventListener('change',updateFunction);
    field.addEventListener('change',updateFunction);
    imode.addEventListener('change',updateFunction);
    instance.addEventListener('change',updateFunction);
    xdrName.addEventListener('change',updateFunction);
    data.addEventListener('change',changeFunction);
    return data;
}

function isXdrUsed(element){
    let parts=element.value.split(',');
    if (! parts[1]) return false;
    if (! parseInt(parts[1])) return false;
    if (! parts[6]) return false;
    return true;
}

function createFilterInput(configItem, frame) {
    let el = addEl('div','filter',frame);
    let ais = createInput({
        type: 'list',
        name: configItem.name + "_ais",
        list: ['aison', 'aisoff']
    }, el);
    let mode = createInput({
        type: 'list',
        name: configItem.name + "_mode",
        list: ['whitelist', 'blacklist']
    }, el);
    let sentences = createInput({
        type: 'text',
        name: configItem.name + "_sentences",
    }, el);
    let data = addEl('input',undefined,el);
    data.setAttribute('type', 'hidden');
    let changeFunction = function () {
        let cv = data.value || "";
        let parts = cv.split(":");
        ais.value = (parts[0] == '0') ? "aisoff" : "aison";
        mode.value = (parts[1] == '0') ? "whitelist" : "blacklist";
        sentences.value = parts[2] || "";
    }
    let updateFunction = function () {
        let nv = (ais.value == 'aison') ? "1" : "0";
        nv += ":";
        nv += (mode.value == 'blacklist') ? "1" : "0";
        nv += ":";
        nv += sentences.value;
        data.value = nv;
        let chev = new Event('change');
        data.dispatchEvent(chev);
    }
    mode.addEventListener('change', updateFunction);
    ais.addEventListener("change", updateFunction);
    sentences.addEventListener("change", updateFunction);
    data.addEventListener('change', function (ev) {
        changeFunction();
    });
    data.setAttribute('name', configItem.name);
    return data;
}
let moreicons=['icon-more','icon-less'];

function collapseCategories(parent,expand){
    let doExpand=expand;
    forEl('.category',function(cat){
        if (typeof(expand) === 'function') doExpand=expand(cat);
        forEl('.content',function(content){
            if (doExpand){
                content.classList.remove('hidden');
            }
            else{
                content.classList.add('hidden');
            }
        },cat);
        forEl('.title .icon',function(icon){
            toggleClass(icon,doExpand?1:0,moreicons);    
        },cat);
    },parent);
}

function findFreeXdr(data){
    let page=document.getElementById('xdrPage');
    let el=undefined;
    collapseCategories(page,function(cat){
        if (el) return false;
        let vEl=cat.querySelector('.xdrvalue');
        if (!vEl) return false;
        if (isXdrUsed(vEl)) return false;
        el=vEl;
        if (data){
            el.value=data;
            let ev=new Event('change');
            el.dispatchEvent(ev);
            window.setTimeout(function(){
                cat.scrollIntoView();
            },50);
        }
        return true;
    });
}

function convertUnassigned(value){
    let rt={};
    value=parseInt(value);
    if (isNaN(value)) return;
    //see GwXDRMappings::addUnknown
    let instance=value & 0x1ff;
    value = value >> 9;
    let field=value & 0x7f;
    value = value >> 7;
    let selector=value & 0x7f;
    value = value >> 7;
    let cid=value & 0x7f;
    let category=getXdrCategory(cid);
    let cname=getXdrCategoryName(cid);
    if (! cname) return rt;
    let fieldName="";
    let idx=0;
    (category.fields || []).forEach(function(f){
        if (f.v === undefined){
            if (idx == field) fieldName=f.l;
        }
        else{
            if (parseInt(f.v) == field) fieldName=f.l;
        }
        idx++;
    });
    let selectorName=selector+"";
    (category.selector ||[]).forEach(function(s){
        if (parseInt(s.v) == selector) selectorName=s.l;
    });
    rt.l=cname+","+selectorName+","+fieldName+","+instance;
    rt.v=cid+",1,"+selector+","+field+",0,"+instance+",";
    return rt;
}

function unassignedAdd(ev) {
    let dv = ev.target.getAttribute('data-value');
    if (dv) {
        findFreeXdr(dv);
        hideOverlay();
    }
}
function loadUnassigned(){
    getText("/api/xdrUnmapped")
        .then(function(txt){
            let ot="";
            txt.split('\n').forEach(function(line){
                let cv=convertUnassigned(line);
                if (!cv || !cv.l) return;
                ot+='<div class="xdrunassigned"><span>'+
                    cv.l+'</span>'+
                    '<button class="addunassigned" data-value="'+
                    cv.v+
                    '">+</button></div>';
            })
            showOverlay(ot,true);
            forEl('.addunassigned',function(bt){
                bt.onclick=unassignedAdd;
            });
        })
}
function showXdrHelp(){
    let helpContent=document.getElementById('xdrHelp');
    if (helpContent){
        showOverlay(helpContent.innerHTML,true);
    }
}
function formatDate(d){
    if (! d) d=new Date();
    let rt=""+d.getFullYear();
    let v=d.getMonth();
    if (v < 10) rt+="0"+v;
    else rt+=v;
    v=d.getDate();
    if (v < 10) rt+="0"+v;
    else rt+=v;
    return rt;
}
function exportXdr(){
    let data={};
    forEl('.xdrvalue',function(el) {
        let name=el.getAttribute('name');
        let value=el.value;
        let err=checkXDR(value,data);
        if (err){
            alert("error in "+name+": "+value+"\n"+err);
            return;
        }
        data[name]=value;
    })
    let url="data:application/octet-stream,"+encodeURIComponent(JSON.stringify(data,undefined,2));
    let target=document.getElementById('downloadXdr');
    if (! target) return;
    target.setAttribute('href',url);
    target.setAttribute('download',"xdr"+formatDate()+".json");
    target.click();
}
function importXdr(){
    forEl('.uploadXdr',function(ul){
        ul.remove();
    });
    let ip=addEl('input','uploadXdr',document.body);
    ip.setAttribute('type','file');
    ip.addEventListener('change',function(ev){
        if (ip.files.length > 0){
            let f=ip.files[0];
            let reader=new FileReader();
            reader.onloadend=function(status){
                try{
                    let idata=JSON.parse(reader.result);
                    let hasOverwrites=false;
                    for (let k in idata){
                        if (! k.match(/^XDR[0-9][0-9]*/)){
                            alert("file contains invalid key "+k);
                            return;
                        }
                        let del=document.querySelector('input[name='+k+']');
                        if (del){
                            hasOverwrites=true;
                        }        
                    }
                    if (hasOverwrites){
                        if (!confirm("overwrite existing data?")) return;
                    }
                    for (let k in idata){
                        let del=document.querySelector('input[name='+k+']');
                        if (del){
                            del.value=idata[k];
                            let ev=new Event('change');
                            del.dispatchEvent(ev);
                        }
                    }
                }catch (error){
                    alert("unable to parse upload: "+error);
                    return;
                }
            };
            reader.readAsBinaryString(f);
        }
        ip.remove();
    });
    ip.click(); 
}
function toggleClass(el,id,classList){
    let nc=classList[id];
    let rt=false;
    if (nc && !el.classList.contains(nc)) rt=true;
    for (let i in classList){
        if (i == id) continue;
        el.classList.remove(classList[i])
    }
    if (nc) el.classList.add(nc);
    return rt;
}

function createConfigDefinitions(parent, capabilities, defs,includeXdr) {
    let category;
    let categoryEl;
    let categoryFrame;
    let frame = parent.querySelector('.configFormRows');
    if (!frame) throw Error("no config form");
    frame.innerHTML = '';
    configDefinitions = defs;
    let currentCategoryPopulated=true;
    defs.forEach(function (item) {
        if (!item.type) return;
        if (item.category.match(/^xdr/)){
            if (! includeXdr) return;
        }
        else{
            if(includeXdr) return;
        } 
        if (item.category != category || !categoryEl) {
            if (categoryFrame && ! currentCategoryPopulated){
                categoryFrame.remove();
            }
            currentCategoryPopulated=false;
            categoryFrame = addEl('div', 'category', frame);
            categoryFrame.setAttribute('data-category',item.category)
            let categoryTitle = addEl('div', 'title', categoryFrame);
            let categoryButton = addEl('span', 'icon icon-more', categoryTitle);
            addEl('span', 'label', categoryTitle, item.category);
            addEl('span','categoryAdd',categoryTitle);
            categoryEl = addEl('div', 'content', categoryFrame);
            categoryEl.classList.add('hidden');
            let currentEl = categoryEl;
            categoryTitle.addEventListener('click', function (ev) {
                let rs = currentEl.classList.toggle('hidden');
                if (rs) {
                    toggleClass(categoryButton,0,moreicons);
                }
                else {
                    toggleClass(categoryButton,1,moreicons);
                }
            })
            category = item.category;
        }
        let showItem=true;
        if (item.capabilities !== undefined) {
            for (let capability in item.capabilities) {
                let values = item.capabilities[capability];
                let found = false;
                if (! (values instanceof Array)) values=[values];
                values.forEach(function (v) {
                    if (capabilities[capability] == v) found = true;
                });
                if (!found) showItem=false;
            }
        }
        if (showItem) {
            currentCategoryPopulated=true;
            let row = addEl('div', 'row', categoryEl);
            let label = item.label || item.name;
            addEl('span', 'label', row, label);
            let valueFrame = addEl('div', 'value', row);
            let valueEl = createInput(item, valueFrame);
            if (!valueEl) return;
            valueEl.setAttribute('data-default', item.default);
            valueEl.addEventListener('change', function (ev) {
                let el = ev.target;
                checkChange(el, row, item.name);
            })
            let condition = getConditions(item.name);
            if (condition) {
                condition.forEach(function (cel) {
                    for (let c in cel) {
                        if (!conditionRelations[c]) {
                            conditionRelations[c] = [];
                        }
                        conditionRelations[c].push(valueEl);
                    }
                })
            }
            if (item.check) valueEl.setAttribute('data-check', item.check);
            let btContainer = addEl('div', 'buttonContainer', row);
            let bt = addEl('button', 'defaultButton', btContainer, 'X');
            bt.setAttribute('data-default', item.default);
            bt.addEventListener('click', function (ev) {
                valueEl.value = valueEl.getAttribute('data-default');
                let changeEvent = new Event('change');
                valueEl.dispatchEvent(changeEvent);
            })
            bt = addEl('button', 'infoButton', btContainer, '?');
            bt.addEventListener('click', function (ev) {
                if (item.description) {
                    showOverlay(item.description);
                }
                else {
                    if (item.category.match(/^xdr/)) {
                        showXdrHelp();
                    }
                }
            });
        }
    });
    if (categoryFrame && ! currentCategoryPopulated){
        categoryFrame.remove();
    }
}
function loadConfigDefinitions() {
    getJson("api/capabilities")
        .then(function (capabilities) {
            getJson("config.json")
                .then(function (defs) {
                    getJson("xdrconfig.json")
                        .then(function(xdr){
                            xdrConfig=xdr;
                            configDefinitions=defs;
                            let normalConfig=document.getElementById('configPage');
                            let xdrParent=document.getElementById('xdrPage');
                            if (normalConfig) createConfigDefinitions(normalConfig,capabilities,defs,false);
                            if (xdrParent) createConfigDefinitions(xdrParent,capabilities,defs,true);
                            resetForm();
                            getText('api/boatDataString').then(function (data) {
                                updateDashboard(data.split('\n'));
                            });
                        })
                })
        })
        .catch(function (err) { alert("unable to load config: " + err) })
}
function converterInfo() {
    getJson("api/converterInfo").then(function (json) {
        let text = "<h3>Converted entities</h3>";
        text += "<p><b>NMEA0183 to NMEA2000:</b><br/>";
        text += "   " + (json.nmea0183 || "").replace(/,/g, ", ");
        text += "</p>";
        text += "<p><b>NMEA2000 to NMEA0183:</b><br/>";
        text += "   " + (json.nmea2000 || "").replace(/,/g, ", ");
        text += "</p>";
        showOverlay(text, true);
    });
}
function handleTab(el) {
    let activeName = el.getAttribute('data-page');
    if (!activeName) return;
    let activeTab = document.getElementById(activeName);
    if (!activeTab) return;
    let all = document.querySelectorAll('.tabPage');
    for (let i = 0; i < all.length; i++) {
        all[i].classList.add('hidden');
    }
    let tabs = document.querySelectorAll('.tab');
    for (let i = 0; i < all.length; i++) {
        tabs[i].classList.remove('active');
    }
    el.classList.add('active');
    activeTab.classList.remove('hidden');
}
/**
 *
 * @param {number} coordinate
 * @param axis
 * @returns {string}
 */
 function formatLonLatsDecimal(coordinate,axis){
    coordinate = (coordinate+540)%360 - 180; // normalize for sphere being round

    let abscoordinate = Math.abs(coordinate);
    let coordinatedegrees = Math.floor(abscoordinate);

    let coordinateminutes = (abscoordinate - coordinatedegrees)/(1/60);
    let numdecimal=2;
    //correctly handle the toFixed(x) - will do math rounding
    if (coordinateminutes.toFixed(numdecimal) == 60){
        coordinatedegrees+=1;
        coordinateminutes=0;
    }
    if( coordinatedegrees < 10 ) {
        coordinatedegrees = "0" + coordinatedegrees;
    }
    if (coordinatedegrees < 100 && axis == 'lon'){
        coordinatedegrees = "0" + coordinatedegrees;
    }
    let str = coordinatedegrees + "\u00B0";

    if( coordinateminutes < 10 ) {
        str +="0";
    }
    str += coordinateminutes.toFixed(numdecimal) + "'";
    if (axis == "lon") {
        str += coordinate < 0 ? "W" :"E";
    } else {
        str += coordinate < 0 ? "S" :"N";
    }
    return str;
};
let valueFormatters = {
    formatCourse: {
        f: function (v) {
            let x = parseFloat(v);
            let rt = x * 180.0 / Math.PI;
            if (rt > 360) rt -= 360;
            if (rt < 0) rt += 360;
            return rt.toFixed(0);
        },
        u: '°'
    },
    formatKnots: {
        f: function (v) {
            let x = parseFloat(v);
            x = x * 3600.0 / 1852.0;
            return x.toFixed(2);
        },
        u: 'kn'
    },
    formatWind: {
        f: function (v) {
            let x = parseFloat(v);
            x = x * 180.0 / Math.PI;
            if (x > 180) x = 180 - x;
            return x.toFixed(0);
        },
        u: '°'
    },
    mtr2nm: {
        f: function (v) {
            let x = parseFloat(v);
            x = x / 1852.0;
            return x.toFixed(2);
        },
        u: 'nm'
    },
    kelvinToC: {
        f: function (v) {
            let x = parseFloat(v);
            x = x - 273.15;
            return x.toFixed(0);
        },
        u: '°'
    },
    formatFixed0: {
        f: function (v) {
            let x = parseFloat(v);
            return x.toFixed(0);
        },
        u: ''
    },
    formatDepth: {
        f: function (v) {
            let x = parseFloat(v);
            return x.toFixed(1);
        },
        u: 'm'
    },
    formatLatitude: {
        f: function (v) {
            let x = parseFloat(v);
            if (isNaN(x)) return '-----';
            return formatLonLatsDecimal(x,'lat');
        },
        u: '°'
    },
    formatLongitude: {
        f: function (v) {
            let x = parseFloat(v);
            if (isNaN(x)) return '-----';
            return formatLonLatsDecimal(x,'lon');
        },
        u: ''
    },
    formatRot:{
        f: function(v){
            let x=parseFloat(v);
            if (isNaN(x)) return '---';
            x = x * 180.0 / Math.PI;
            return x.toFixed(2);
        },
        u:'°/s'
    },
    formatXte:{
        f: function(v){
            let x=parseFloat(v);
            if (isNaN(x)) return '---';
            return x.toFixed(0);
        },
        u:'m'
    }
}
function resizeFont(el,reset,maxIt){
    if (maxIt === undefined) maxIt=10;
    if (! el) return;
    if (reset) el.style.fontSize='';
    while (el.scrollWidth > el.clientWidth && maxIt){
        let next=parseFloat(window.getComputedStyle(el).fontSize)*0.9;
        el.style.fontSize=next+"px";
    }
}
function createDashboardItem(name, def, parent) {
    if (! def.name) return;
    let frame = addEl('div', 'dash', parent);
    let title = addEl('span', 'dashTitle', frame, name);
    let value = addEl('span', 'dashValue', frame);
    value.setAttribute('id', 'data_' + name);
    let fmt=valueFormatters[def.format];
    if (def.format) value.classList.add(def.format);
    let footer = addEl('div','footer',frame);
    let src= addEl('span','source',footer);
    src.setAttribute('id','source_'+name);
    let u=fmt?fmt.u:' ';
    if (! fmt && def.format && def.format.match(/formatXdr/)){
        u=def.format.replace(/formatXdr/,'');
    }
    addEl('span','unit',footer,u);
    return value;
}
function parseBoatDataLine(line){
    let rt={};
    let parts=line.split(',');
    rt.name=parts[0];
    rt.valid=parts[1] === '1';
    rt.update=parseInt(parts[2]);
    rt.source=parseInt(parts[3]);
    rt.format=parts[4];
    rt.value=parts[5];
    return rt;
}
function createDashboard() {
    let frame = document.getElementById('dashboardPage');
    if (!frame) return;
    getText("api/boatDataString").then(function (txt) {
        frame.innerHTML = '';
        let values=txt.split('\n');
        for (let n in values) {
            let def=parseBoatDataLine(values[n]);
            createDashboardItem(def.name, def, frame);
        }
        updateDashboard(values);
    });
}
function sourceName(v){
    if (v == 0) return "N2K";
    if (v == 1) return "USB";
    if (v == 2) return "SER";
    if (v >= 3) return "TCP";
    return "---";
}
let lastSelectList=[];
function updateDashboard(data) {
    let frame = document.getElementById('dashboardPage');
    let names={};
    for (let n in data) {
        let current=parseBoatDataLine(data[n]);
        if (! current.name) continue;
        names[current.name]=true;
        let de = document.getElementById('data_' + current.name);
        if (! de && frame){
            de=createDashboardItem(current.name,current,frame);   
        }
        if (de) {
            let newContent='----';
            if (current.valid) {
                let formatter;
                if (current.format && current.format != "NULL") {
                    let key = current.format.replace(/^\&/, '');
                    formatter = valueFormatters[key];
                }
                if (formatter) {
                    newContent = formatter.f(current.value);
                }
                else {
                    let v = parseFloat(current.value);
                    if (!isNaN(v)) {
                        v = v.toFixed(3)
                        newContent = v;
                    }
                    else {
                        newContent = current.value;
                    }
                }
            }
            else newContent = "---";
            if (newContent !== de.textContent){
                de.textContent=newContent;
                resizeFont(de,true);
            }
        }
        let src=document.getElementById('source_'+current.name);
        if (src){
            src.textContent=sourceName(current.source);
        }
    }
    console.log("update");
    forEl('.dashValue',function(el){
        let id=el.getAttribute('id');
        if (id){
            if (! names[id.replace(/^data_/,'')]){
                el.parentElement.remove();
            }
        }
    });
    let selectList=[];
    for (let n in names){
        selectList.push({l:n,v:n});
    }
    let selectChanged=false;
    if (lastSelectList.length == selectList.length){
        for (let i=0;i<lastSelectList.length;i++){
            if (selectList[i] != lastSelectList[i]){
                selectChanged=true;
                break;
            }
        }
    }
    else{
        selectChanged=true;
    }
    if (selectChanged){
        forEl('.boatDataSelect',function(el){
            updateSelectList(el,selectList);
        });
    }
}

window.setInterval(update, 1000);
window.setInterval(function () {
    let dp = document.getElementById('dashboardPage');
    if (dp.classList.contains('hidden')) return;
    getText('api/boatDataString').then(function (data) {
        updateDashboard(data.split('\n'));
    });
}, 1000);
window.addEventListener('load', function () {
    let buttons = document.querySelectorAll('button');
    for (let i = 0; i < buttons.length; i++) {
        let be = buttons[i];
        be.onclick = window[be.id]; //assume a function with the button id
    }
    forEl('.showMsgDetails', function (cd) {
        cd.addEventListener('change', function (ev) {
            let key = ev.target.getAttribute('data-key');
            if (!key) return;
            let el = document.getElementById(key);
            if (!el) return;
            if (ev.target.checked) el.classList.remove('hidden');
            else (el.classList).add('hidden');
        });
    });
    let tabs = document.querySelectorAll('.tab');
    for (let i = 0; i < tabs.length; i++) {
        tabs[i].addEventListener('click', function (ev) {
            handleTab(ev.target);
        });
    }
    loadConfigDefinitions();
    createDashboard();
    let statusPage=document.getElementById('statusPageContent');
    if (statusPage){
        let even=true;
        for (let c in counters){
            createCounterDisplay(statusPage,counters[c],c,even);
            even=!even;
        }
    }
});