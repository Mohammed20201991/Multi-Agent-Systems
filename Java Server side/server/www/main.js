var Massim = (function (exports) {
    'use strict';

    function vnode(sel, data, children, text, elm) {
        var key = data === undefined ? undefined : data.key;
        return { sel: sel, data: data, children: children, text: text, elm: elm, key: key };
    }

    var array = Array.isArray;
    function primitive(s) {
        return typeof s === 'string' || typeof s === 'number';
    }

    function createElement(tagName) {
        return document.createElement(tagName);
    }
    function createElementNS(namespaceURI, qualifiedName) {
        return document.createElementNS(namespaceURI, qualifiedName);
    }
    function createTextNode(text) {
        return document.createTextNode(text);
    }
    function createComment(text) {
        return document.createComment(text);
    }
    function insertBefore(parentNode, newNode, referenceNode) {
        parentNode.insertBefore(newNode, referenceNode);
    }
    function removeChild(node, child) {
        node.removeChild(child);
    }
    function appendChild(node, child) {
        node.appendChild(child);
    }
    function parentNode(node) {
        return node.parentNode;
    }
    function nextSibling(node) {
        return node.nextSibling;
    }
    function tagName(elm) {
        return elm.tagName;
    }
    function setTextContent(node, text) {
        node.textContent = text;
    }
    function getTextContent(node) {
        return node.textContent;
    }
    function isElement(node) {
        return node.nodeType === 1;
    }
    function isText(node) {
        return node.nodeType === 3;
    }
    function isComment(node) {
        return node.nodeType === 8;
    }
    var htmlDomApi = {
        createElement: createElement,
        createElementNS: createElementNS,
        createTextNode: createTextNode,
        createComment: createComment,
        insertBefore: insertBefore,
        removeChild: removeChild,
        appendChild: appendChild,
        parentNode: parentNode,
        nextSibling: nextSibling,
        tagName: tagName,
        setTextContent: setTextContent,
        getTextContent: getTextContent,
        isElement: isElement,
        isText: isText,
        isComment: isComment,
    };

    function addNS(data, children, sel) {
        data.ns = 'http://www.w3.org/2000/svg';
        if (sel !== 'foreignObject' && children !== undefined) {
            for (var i = 0; i < children.length; ++i) {
                var childData = children[i].data;
                if (childData !== undefined) {
                    addNS(childData, children[i].children, children[i].sel);
                }
            }
        }
    }
    function h(sel, b, c) {
        var data = {}, children, text, i;
        if (c !== undefined) {
            data = b;
            if (array(c)) {
                children = c;
            }
            else if (primitive(c)) {
                text = c;
            }
            else if (c && c.sel) {
                children = [c];
            }
        }
        else if (b !== undefined) {
            if (array(b)) {
                children = b;
            }
            else if (primitive(b)) {
                text = b;
            }
            else if (b && b.sel) {
                children = [b];
            }
            else {
                data = b;
            }
        }
        if (children !== undefined) {
            for (i = 0; i < children.length; ++i) {
                if (primitive(children[i]))
                    children[i] = vnode(undefined, undefined, undefined, children[i], undefined);
            }
        }
        if (sel[0] === 's' && sel[1] === 'v' && sel[2] === 'g' &&
            (sel.length === 3 || sel[3] === '.' || sel[3] === '#')) {
            addNS(data, children, sel);
        }
        return vnode(sel, data, children, text, undefined);
    }

    function isUndef(s) { return s === undefined; }
    function isDef(s) { return s !== undefined; }
    var emptyNode = vnode('', {}, [], undefined, undefined);
    function sameVnode(vnode1, vnode2) {
        return vnode1.key === vnode2.key && vnode1.sel === vnode2.sel;
    }
    function isVnode(vnode) {
        return vnode.sel !== undefined;
    }
    function createKeyToOldIdx(children, beginIdx, endIdx) {
        var i, map = {}, key, ch;
        for (i = beginIdx; i <= endIdx; ++i) {
            ch = children[i];
            if (ch != null) {
                key = ch.key;
                if (key !== undefined)
                    map[key] = i;
            }
        }
        return map;
    }
    var hooks = ['create', 'update', 'remove', 'destroy', 'pre', 'post'];
    function init(modules, domApi) {
        var i, j, cbs = {};
        var api = domApi !== undefined ? domApi : htmlDomApi;
        for (i = 0; i < hooks.length; ++i) {
            cbs[hooks[i]] = [];
            for (j = 0; j < modules.length; ++j) {
                var hook = modules[j][hooks[i]];
                if (hook !== undefined) {
                    cbs[hooks[i]].push(hook);
                }
            }
        }
        function emptyNodeAt(elm) {
            var id = elm.id ? '#' + elm.id : '';
            var c = elm.className ? '.' + elm.className.split(' ').join('.') : '';
            return vnode(api.tagName(elm).toLowerCase() + id + c, {}, [], undefined, elm);
        }
        function createRmCb(childElm, listeners) {
            return function rmCb() {
                if (--listeners === 0) {
                    var parent_1 = api.parentNode(childElm);
                    api.removeChild(parent_1, childElm);
                }
            };
        }
        function createElm(vnode, insertedVnodeQueue) {
            var i, data = vnode.data;
            if (data !== undefined) {
                if (isDef(i = data.hook) && isDef(i = i.init)) {
                    i(vnode);
                    data = vnode.data;
                }
            }
            var children = vnode.children, sel = vnode.sel;
            if (sel === '!') {
                if (isUndef(vnode.text)) {
                    vnode.text = '';
                }
                vnode.elm = api.createComment(vnode.text);
            }
            else if (sel !== undefined) {
                // Parse selector
                var hashIdx = sel.indexOf('#');
                var dotIdx = sel.indexOf('.', hashIdx);
                var hash = hashIdx > 0 ? hashIdx : sel.length;
                var dot = dotIdx > 0 ? dotIdx : sel.length;
                var tag = hashIdx !== -1 || dotIdx !== -1 ? sel.slice(0, Math.min(hash, dot)) : sel;
                var elm = vnode.elm = isDef(data) && isDef(i = data.ns) ? api.createElementNS(i, tag)
                    : api.createElement(tag);
                if (hash < dot)
                    elm.setAttribute('id', sel.slice(hash + 1, dot));
                if (dotIdx > 0)
                    elm.setAttribute('class', sel.slice(dot + 1).replace(/\./g, ' '));
                for (i = 0; i < cbs.create.length; ++i)
                    cbs.create[i](emptyNode, vnode);
                if (array(children)) {
                    for (i = 0; i < children.length; ++i) {
                        var ch = children[i];
                        if (ch != null) {
                            api.appendChild(elm, createElm(ch, insertedVnodeQueue));
                        }
                    }
                }
                else if (primitive(vnode.text)) {
                    api.appendChild(elm, api.createTextNode(vnode.text));
                }
                i = vnode.data.hook; // Reuse variable
                if (isDef(i)) {
                    if (i.create)
                        i.create(emptyNode, vnode);
                    if (i.insert)
                        insertedVnodeQueue.push(vnode);
                }
            }
            else {
                vnode.elm = api.createTextNode(vnode.text);
            }
            return vnode.elm;
        }
        function addVnodes(parentElm, before, vnodes, startIdx, endIdx, insertedVnodeQueue) {
            for (; startIdx <= endIdx; ++startIdx) {
                var ch = vnodes[startIdx];
                if (ch != null) {
                    api.insertBefore(parentElm, createElm(ch, insertedVnodeQueue), before);
                }
            }
        }
        function invokeDestroyHook(vnode) {
            var i, j, data = vnode.data;
            if (data !== undefined) {
                if (isDef(i = data.hook) && isDef(i = i.destroy))
                    i(vnode);
                for (i = 0; i < cbs.destroy.length; ++i)
                    cbs.destroy[i](vnode);
                if (vnode.children !== undefined) {
                    for (j = 0; j < vnode.children.length; ++j) {
                        i = vnode.children[j];
                        if (i != null && typeof i !== "string") {
                            invokeDestroyHook(i);
                        }
                    }
                }
            }
        }
        function removeVnodes(parentElm, vnodes, startIdx, endIdx) {
            for (; startIdx <= endIdx; ++startIdx) {
                var i_1 = void 0, listeners = void 0, rm = void 0, ch = vnodes[startIdx];
                if (ch != null) {
                    if (isDef(ch.sel)) {
                        invokeDestroyHook(ch);
                        listeners = cbs.remove.length + 1;
                        rm = createRmCb(ch.elm, listeners);
                        for (i_1 = 0; i_1 < cbs.remove.length; ++i_1)
                            cbs.remove[i_1](ch, rm);
                        if (isDef(i_1 = ch.data) && isDef(i_1 = i_1.hook) && isDef(i_1 = i_1.remove)) {
                            i_1(ch, rm);
                        }
                        else {
                            rm();
                        }
                    }
                    else { // Text node
                        api.removeChild(parentElm, ch.elm);
                    }
                }
            }
        }
        function updateChildren(parentElm, oldCh, newCh, insertedVnodeQueue) {
            var oldStartIdx = 0, newStartIdx = 0;
            var oldEndIdx = oldCh.length - 1;
            var oldStartVnode = oldCh[0];
            var oldEndVnode = oldCh[oldEndIdx];
            var newEndIdx = newCh.length - 1;
            var newStartVnode = newCh[0];
            var newEndVnode = newCh[newEndIdx];
            var oldKeyToIdx;
            var idxInOld;
            var elmToMove;
            var before;
            while (oldStartIdx <= oldEndIdx && newStartIdx <= newEndIdx) {
                if (oldStartVnode == null) {
                    oldStartVnode = oldCh[++oldStartIdx]; // Vnode might have been moved left
                }
                else if (oldEndVnode == null) {
                    oldEndVnode = oldCh[--oldEndIdx];
                }
                else if (newStartVnode == null) {
                    newStartVnode = newCh[++newStartIdx];
                }
                else if (newEndVnode == null) {
                    newEndVnode = newCh[--newEndIdx];
                }
                else if (sameVnode(oldStartVnode, newStartVnode)) {
                    patchVnode(oldStartVnode, newStartVnode, insertedVnodeQueue);
                    oldStartVnode = oldCh[++oldStartIdx];
                    newStartVnode = newCh[++newStartIdx];
                }
                else if (sameVnode(oldEndVnode, newEndVnode)) {
                    patchVnode(oldEndVnode, newEndVnode, insertedVnodeQueue);
                    oldEndVnode = oldCh[--oldEndIdx];
                    newEndVnode = newCh[--newEndIdx];
                }
                else if (sameVnode(oldStartVnode, newEndVnode)) { // Vnode moved right
                    patchVnode(oldStartVnode, newEndVnode, insertedVnodeQueue);
                    api.insertBefore(parentElm, oldStartVnode.elm, api.nextSibling(oldEndVnode.elm));
                    oldStartVnode = oldCh[++oldStartIdx];
                    newEndVnode = newCh[--newEndIdx];
                }
                else if (sameVnode(oldEndVnode, newStartVnode)) { // Vnode moved left
                    patchVnode(oldEndVnode, newStartVnode, insertedVnodeQueue);
                    api.insertBefore(parentElm, oldEndVnode.elm, oldStartVnode.elm);
                    oldEndVnode = oldCh[--oldEndIdx];
                    newStartVnode = newCh[++newStartIdx];
                }
                else {
                    if (oldKeyToIdx === undefined) {
                        oldKeyToIdx = createKeyToOldIdx(oldCh, oldStartIdx, oldEndIdx);
                    }
                    idxInOld = oldKeyToIdx[newStartVnode.key];
                    if (isUndef(idxInOld)) { // New element
                        api.insertBefore(parentElm, createElm(newStartVnode, insertedVnodeQueue), oldStartVnode.elm);
                        newStartVnode = newCh[++newStartIdx];
                    }
                    else {
                        elmToMove = oldCh[idxInOld];
                        if (elmToMove.sel !== newStartVnode.sel) {
                            api.insertBefore(parentElm, createElm(newStartVnode, insertedVnodeQueue), oldStartVnode.elm);
                        }
                        else {
                            patchVnode(elmToMove, newStartVnode, insertedVnodeQueue);
                            oldCh[idxInOld] = undefined;
                            api.insertBefore(parentElm, elmToMove.elm, oldStartVnode.elm);
                        }
                        newStartVnode = newCh[++newStartIdx];
                    }
                }
            }
            if (oldStartIdx <= oldEndIdx || newStartIdx <= newEndIdx) {
                if (oldStartIdx > oldEndIdx) {
                    before = newCh[newEndIdx + 1] == null ? null : newCh[newEndIdx + 1].elm;
                    addVnodes(parentElm, before, newCh, newStartIdx, newEndIdx, insertedVnodeQueue);
                }
                else {
                    removeVnodes(parentElm, oldCh, oldStartIdx, oldEndIdx);
                }
            }
        }
        function patchVnode(oldVnode, vnode, insertedVnodeQueue) {
            var i, hook;
            if (isDef(i = vnode.data) && isDef(hook = i.hook) && isDef(i = hook.prepatch)) {
                i(oldVnode, vnode);
            }
            var elm = vnode.elm = oldVnode.elm;
            var oldCh = oldVnode.children;
            var ch = vnode.children;
            if (oldVnode === vnode)
                return;
            if (vnode.data !== undefined) {
                for (i = 0; i < cbs.update.length; ++i)
                    cbs.update[i](oldVnode, vnode);
                i = vnode.data.hook;
                if (isDef(i) && isDef(i = i.update))
                    i(oldVnode, vnode);
            }
            if (isUndef(vnode.text)) {
                if (isDef(oldCh) && isDef(ch)) {
                    if (oldCh !== ch)
                        updateChildren(elm, oldCh, ch, insertedVnodeQueue);
                }
                else if (isDef(ch)) {
                    if (isDef(oldVnode.text))
                        api.setTextContent(elm, '');
                    addVnodes(elm, null, ch, 0, ch.length - 1, insertedVnodeQueue);
                }
                else if (isDef(oldCh)) {
                    removeVnodes(elm, oldCh, 0, oldCh.length - 1);
                }
                else if (isDef(oldVnode.text)) {
                    api.setTextContent(elm, '');
                }
            }
            else if (oldVnode.text !== vnode.text) {
                if (isDef(oldCh)) {
                    removeVnodes(elm, oldCh, 0, oldCh.length - 1);
                }
                api.setTextContent(elm, vnode.text);
            }
            if (isDef(hook) && isDef(i = hook.postpatch)) {
                i(oldVnode, vnode);
            }
        }
        return function patch(oldVnode, vnode) {
            var i, elm, parent;
            var insertedVnodeQueue = [];
            for (i = 0; i < cbs.pre.length; ++i)
                cbs.pre[i]();
            if (!isVnode(oldVnode)) {
                oldVnode = emptyNodeAt(oldVnode);
            }
            if (sameVnode(oldVnode, vnode)) {
                patchVnode(oldVnode, vnode, insertedVnodeQueue);
            }
            else {
                elm = oldVnode.elm;
                parent = api.parentNode(elm);
                createElm(vnode, insertedVnodeQueue);
                if (parent !== null) {
                    api.insertBefore(parent, vnode.elm, api.nextSibling(elm));
                    removeVnodes(parent, [oldVnode], 0, 0);
                }
            }
            for (i = 0; i < insertedVnodeQueue.length; ++i) {
                insertedVnodeQueue[i].data.hook.insert(insertedVnodeQueue[i]);
            }
            for (i = 0; i < cbs.post.length; ++i)
                cbs.post[i]();
            return vnode;
        };
    }

    function createCommonjsModule(fn, basedir, module) {
    	return module = {
    	  path: basedir,
    	  exports: {},
    	  require: function (path, base) {
          return commonjsRequire(path, (base === undefined || base === null) ? module.path : base);
        }
    	}, fn(module, module.exports), module.exports;
    }

    function commonjsRequire () {
    	throw new Error('Dynamic requires are not currently supported by @rollup/plugin-commonjs');
    }

    var _class = createCommonjsModule(function (module, exports) {
    Object.defineProperty(exports, "__esModule", { value: true });
    function updateClass(oldVnode, vnode) {
        var cur, name, elm = vnode.elm, oldClass = oldVnode.data.class, klass = vnode.data.class;
        if (!oldClass && !klass)
            return;
        if (oldClass === klass)
            return;
        oldClass = oldClass || {};
        klass = klass || {};
        for (name in oldClass) {
            if (!klass[name]) {
                elm.classList.remove(name);
            }
        }
        for (name in klass) {
            cur = klass[name];
            if (cur !== oldClass[name]) {
                elm.classList[cur ? 'add' : 'remove'](name);
            }
        }
    }
    exports.classModule = { create: updateClass, update: updateClass };
    exports.default = exports.classModule;

    });

    var props = createCommonjsModule(function (module, exports) {
    Object.defineProperty(exports, "__esModule", { value: true });
    function updateProps(oldVnode, vnode) {
        var key, cur, old, elm = vnode.elm, oldProps = oldVnode.data.props, props = vnode.data.props;
        if (!oldProps && !props)
            return;
        if (oldProps === props)
            return;
        oldProps = oldProps || {};
        props = props || {};
        for (key in oldProps) {
            if (!props[key]) {
                delete elm[key];
            }
        }
        for (key in props) {
            cur = props[key];
            old = oldProps[key];
            if (old !== cur && (key !== 'value' || elm[key] !== cur)) {
                elm[key] = cur;
            }
        }
    }
    exports.propsModule = { create: updateProps, update: updateProps };
    exports.default = exports.propsModule;

    });

    var attributes = createCommonjsModule(function (module, exports) {
    Object.defineProperty(exports, "__esModule", { value: true });
    var xlinkNS = 'http://www.w3.org/1999/xlink';
    var xmlNS = 'http://www.w3.org/XML/1998/namespace';
    var colonChar = 58;
    var xChar = 120;
    function updateAttrs(oldVnode, vnode) {
        var key, elm = vnode.elm, oldAttrs = oldVnode.data.attrs, attrs = vnode.data.attrs;
        if (!oldAttrs && !attrs)
            return;
        if (oldAttrs === attrs)
            return;
        oldAttrs = oldAttrs || {};
        attrs = attrs || {};
        // update modified attributes, add new attributes
        for (key in attrs) {
            var cur = attrs[key];
            var old = oldAttrs[key];
            if (old !== cur) {
                if (cur === true) {
                    elm.setAttribute(key, "");
                }
                else if (cur === false) {
                    elm.removeAttribute(key);
                }
                else {
                    if (key.charCodeAt(0) !== xChar) {
                        elm.setAttribute(key, cur);
                    }
                    else if (key.charCodeAt(3) === colonChar) {
                        // Assume xml namespace
                        elm.setAttributeNS(xmlNS, key, cur);
                    }
                    else if (key.charCodeAt(5) === colonChar) {
                        // Assume xlink namespace
                        elm.setAttributeNS(xlinkNS, key, cur);
                    }
                    else {
                        elm.setAttribute(key, cur);
                    }
                }
            }
        }
        // remove removed attributes
        // use `in` operator since the previous `for` iteration uses it (.i.e. add even attributes with undefined value)
        // the other option is to remove all attributes with value == undefined
        for (key in oldAttrs) {
            if (!(key in attrs)) {
                elm.removeAttribute(key);
            }
        }
    }
    exports.attributesModule = { create: updateAttrs, update: updateAttrs };
    exports.default = exports.attributesModule;

    });

    var eventlisteners = createCommonjsModule(function (module, exports) {
    Object.defineProperty(exports, "__esModule", { value: true });
    function invokeHandler(handler, vnode, event) {
        if (typeof handler === "function") {
            // call function handler
            handler.call(vnode, event, vnode);
        }
        else if (typeof handler === "object") {
            // call handler with arguments
            if (typeof handler[0] === "function") {
                // special case for single argument for performance
                if (handler.length === 2) {
                    handler[0].call(vnode, handler[1], event, vnode);
                }
                else {
                    var args = handler.slice(1);
                    args.push(event);
                    args.push(vnode);
                    handler[0].apply(vnode, args);
                }
            }
            else {
                // call multiple handlers
                for (var i = 0; i < handler.length; i++) {
                    invokeHandler(handler[i], vnode, event);
                }
            }
        }
    }
    function handleEvent(event, vnode) {
        var name = event.type, on = vnode.data.on;
        // call event handler(s) if exists
        if (on && on[name]) {
            invokeHandler(on[name], vnode, event);
        }
    }
    function createListener() {
        return function handler(event) {
            handleEvent(event, handler.vnode);
        };
    }
    function updateEventListeners(oldVnode, vnode) {
        var oldOn = oldVnode.data.on, oldListener = oldVnode.listener, oldElm = oldVnode.elm, on = vnode && vnode.data.on, elm = (vnode && vnode.elm), name;
        // optimization for reused immutable handlers
        if (oldOn === on) {
            return;
        }
        // remove existing listeners which no longer used
        if (oldOn && oldListener) {
            // if element changed or deleted we remove all existing listeners unconditionally
            if (!on) {
                for (name in oldOn) {
                    // remove listener if element was changed or existing listeners removed
                    oldElm.removeEventListener(name, oldListener, false);
                }
            }
            else {
                for (name in oldOn) {
                    // remove listener if existing listener removed
                    if (!on[name]) {
                        oldElm.removeEventListener(name, oldListener, false);
                    }
                }
            }
        }
        // add new listeners which has not already attached
        if (on) {
            // reuse existing listener or create new
            var listener = vnode.listener = oldVnode.listener || createListener();
            // update vnode for listener
            listener.vnode = vnode;
            // if element changed or added we add all needed listeners unconditionally
            if (!oldOn) {
                for (name in on) {
                    // add listener if element was changed or new listeners added
                    elm.addEventListener(name, listener, false);
                }
            }
            else {
                for (name in on) {
                    // add listener if new listener added
                    if (!oldOn[name]) {
                        elm.addEventListener(name, listener, false);
                    }
                }
            }
        }
    }
    exports.eventListenersModule = {
        create: updateEventListeners,
        update: updateEventListeners,
        destroy: updateEventListeners
    };
    exports.default = exports.eventListenersModule;

    });

    var style = createCommonjsModule(function (module, exports) {
    Object.defineProperty(exports, "__esModule", { value: true });
    // Bindig `requestAnimationFrame` like this fixes a bug in IE/Edge. See #360 and #409.
    var raf = (typeof window !== 'undefined' && (window.requestAnimationFrame).bind(window)) || setTimeout;
    var nextFrame = function (fn) { raf(function () { raf(fn); }); };
    var reflowForced = false;
    function setNextFrame(obj, prop, val) {
        nextFrame(function () { obj[prop] = val; });
    }
    function updateStyle(oldVnode, vnode) {
        var cur, name, elm = vnode.elm, oldStyle = oldVnode.data.style, style = vnode.data.style;
        if (!oldStyle && !style)
            return;
        if (oldStyle === style)
            return;
        oldStyle = oldStyle || {};
        style = style || {};
        var oldHasDel = 'delayed' in oldStyle;
        for (name in oldStyle) {
            if (!style[name]) {
                if (name[0] === '-' && name[1] === '-') {
                    elm.style.removeProperty(name);
                }
                else {
                    elm.style[name] = '';
                }
            }
        }
        for (name in style) {
            cur = style[name];
            if (name === 'delayed' && style.delayed) {
                for (var name2 in style.delayed) {
                    cur = style.delayed[name2];
                    if (!oldHasDel || cur !== oldStyle.delayed[name2]) {
                        setNextFrame(elm.style, name2, cur);
                    }
                }
            }
            else if (name !== 'remove' && cur !== oldStyle[name]) {
                if (name[0] === '-' && name[1] === '-') {
                    elm.style.setProperty(name, cur);
                }
                else {
                    elm.style[name] = cur;
                }
            }
        }
    }
    function applyDestroyStyle(vnode) {
        var style, name, elm = vnode.elm, s = vnode.data.style;
        if (!s || !(style = s.destroy))
            return;
        for (name in style) {
            elm.style[name] = style[name];
        }
    }
    function applyRemoveStyle(vnode, rm) {
        var s = vnode.data.style;
        if (!s || !s.remove) {
            rm();
            return;
        }
        if (!reflowForced) {
            vnode.elm.offsetLeft;
            reflowForced = true;
        }
        var name, elm = vnode.elm, i = 0, compStyle, style = s.remove, amount = 0, applied = [];
        for (name in style) {
            applied.push(name);
            elm.style[name] = style[name];
        }
        compStyle = getComputedStyle(elm);
        var props = compStyle['transition-property'].split(', ');
        for (; i < props.length; ++i) {
            if (applied.indexOf(props[i]) !== -1)
                amount++;
        }
        elm.addEventListener('transitionend', function (ev) {
            if (ev.target === elm)
                --amount;
            if (amount === 0)
                rm();
        });
    }
    function forceReflow() {
        reflowForced = false;
    }
    exports.styleModule = {
        pre: forceReflow,
        create: updateStyle,
        update: updateStyle,
        destroy: applyDestroyStyle,
        remove: applyRemoveStyle
    };
    exports.default = exports.styleModule;

    });

    function compareAgent(a, b) {
        if (a.team < b.team)
            return -1;
        else if (a.team > b.team)
            return 1;
        const suffixA = parseInt(a.name.replace(/^[^\d]*/, ''), 10);
        const suffixB = parseInt(b.name.replace(/^[^\d]*/, ''), 10);
        if (suffixA < suffixB)
            return -1;
        else if (suffixA > suffixB)
            return 1;
        if (a.name < b.name)
            return -1;
        else if (a.name > b.name)
            return 1;
        else
            return 0;
    }

    const teams = ['blue', 'green', '#ff1493', '#8b0000'];
    const goal = 'rgba(255, 0, 0, 0.4)';
    const goalOnLight = '#f58f8f';
    const obstacle = '#333';
    const board = '#00ffff';
    const blocks = ['#41470b', '#78730d', '#bab217', '#e3d682', '#b3a06f', '#9c7640', '#5a4c35'];
    const hover = 'rgba(180, 180, 255, 0.4)';

    const minScale = 10;
    const maxScale = 100;
    class MapCtrl {
        constructor(root) {
            this.root = root;
            this.vm = {
                transform: {
                    x: 0,
                    y: 0,
                    scale: 20,
                },
            };
        }
        selectedAgent() {
            if (!this.root.vm.dynamic)
                return;
            return this.root.vm.dynamic.entities.find(a => a.id === this.vm.selected);
        }
        select(pos) {
            if (pos && this.root.vm.dynamic) {
                const agents = this.root.vm.dynamic.entities.filter(a => a.x == pos.x && a.y == pos.y);
                agents.reverse(); // opposite of rendering order
                if (agents.every(a => a.id !== this.vm.selected))
                    this.vm.selected = undefined;
                const selected = this.selectedAgent();
                for (const agent of agents) {
                    if (!selected || compareAgent(selected, agent) > 0) {
                        this.vm.selected = agent.id;
                        this.root.redraw();
                        return;
                    }
                }
            }
            this.vm.selected = undefined;
            this.root.redraw();
        }
        invPos(pos, bounds) {
            // relative to bounds
            const x = pos[0] - bounds.x;
            const y = pos[1] - bounds.y;
            if (x < 0 || x > bounds.width || y < 0 || y > bounds.height)
                return;
            // relative to transform
            const p = {
                x: Math.floor((x - this.vm.transform.x) / this.vm.transform.scale),
                y: Math.floor((y - this.vm.transform.y) / this.vm.transform.scale),
            };
            // relative to grid
            if (this.root.vm.static) {
                return {
                    x: mod(p.x, this.root.vm.static.grid.width),
                    y: mod(p.y, this.root.vm.static.grid.height),
                };
            }
            else
                return p;
        }
        zoom(center, factor) {
            if (this.vm.transform.scale * factor < minScale)
                factor = minScale / this.vm.transform.scale;
            if (this.vm.transform.scale * factor > maxScale)
                factor = maxScale / this.vm.transform.scale;
            this.vm.transform = {
                x: center[0] + (this.vm.transform.x - center[0]) * factor,
                y: center[1] + (this.vm.transform.y - center[1]) * factor,
                scale: this.vm.transform.scale * factor,
            };
        }
    }
    function mapView(ctrl, opts) {
        return h('canvas', {
            attrs: (opts === null || opts === void 0 ? void 0 : opts.size) ? {
                width: opts.size,
                height: opts.size,
            } : undefined,
            hook: {
                insert(vnode) {
                    const elm = vnode.elm;
                    if (opts === null || opts === void 0 ? void 0 : opts.size)
                        render(elm, ctrl, opts);
                    else
                        new window.ResizeObserver((entries) => {
                            for (const entry of entries) {
                                elm.width = entry.contentRect.width;
                                elm.height = entry.contentRect.height;
                                requestAnimationFrame(() => render(elm, ctrl, opts));
                            }
                        }).observe(elm);
                    const mouseup = (ev) => {
                        if (ctrl.vm.dragging || ctrl.vm.zooming)
                            ev.preventDefault();
                        if (ctrl.vm.dragging && !ctrl.vm.dragging.started) {
                            const pos = eventPosition(ev) || ctrl.vm.dragging.first;
                            ctrl.select(ctrl.invPos(pos, elm.getBoundingClientRect()));
                        }
                        ctrl.vm.dragging = undefined;
                        ctrl.vm.zooming = undefined;
                    };
                    const mousemove = (ev) => {
                        const zoom = eventZoom(ev);
                        if (ctrl.vm.zooming && zoom) {
                            ctrl.vm.transform = Object.assign({}, ctrl.vm.zooming.initialTransform);
                            ctrl.zoom([
                                (ctrl.vm.zooming.zoom.center[0] + zoom.center[0]) / 2,
                                (ctrl.vm.zooming.zoom.center[1] + zoom.center[1]) / 2,
                            ], zoom.distance / ctrl.vm.zooming.zoom.distance);
                            ev.preventDefault();
                            return;
                        }
                        const pos = eventPosition(ev);
                        if (pos) {
                            const inv = ctrl.invPos(pos, elm.getBoundingClientRect());
                            if (inv)
                                ctrl.root.setHover(inv);
                        }
                        if (ctrl.vm.dragging && pos) {
                            if (ctrl.vm.dragging.started || distanceSq(ctrl.vm.dragging.first, pos) > 20 * 20) {
                                ctrl.vm.dragging.started = true;
                                ctrl.vm.transform.x += pos[0] - ctrl.vm.dragging.latest[0];
                                ctrl.vm.transform.y += pos[1] - ctrl.vm.dragging.latest[1];
                                ctrl.vm.dragging.latest = pos;
                            }
                            ev.preventDefault();
                        }
                    };
                    const mousedown = (ev) => {
                        if (ev.button !== undefined && ev.button !== 0)
                            return; // only left click
                        const pos = eventPosition(ev);
                        const zoom = eventZoom(ev);
                        if (zoom) {
                            ctrl.vm.zooming = {
                                initialTransform: Object.assign({}, ctrl.vm.transform),
                                zoom,
                            };
                        }
                        else if (pos) {
                            ctrl.vm.dragging = {
                                first: pos,
                                latest: pos,
                                started: false,
                            };
                        }
                        if (zoom || pos) {
                            ev.preventDefault();
                            requestAnimationFrame(() => render(ev.target, ctrl, opts, true));
                        }
                    };
                    const wheel = (ev) => {
                        ev.preventDefault();
                        ctrl.zoom([ev.offsetX, ev.offsetY], Math.pow(3 / 2, -ev.deltaY / (ev.deltaMode ? 6.25 : 100)));
                        requestAnimationFrame(() => render(ev.target, ctrl, opts));
                    };
                    elm.massim = {
                        unbinds: (opts === null || opts === void 0 ? void 0 : opts.viewOnly) ? [
                            unbindable(document, 'mousemove', mousemove, { passive: false }),
                        ] : [
                            unbindable(elm, 'mousedown', mousedown, { passive: false }),
                            unbindable(elm, 'touchstart', mousedown, { passive: false }),
                            unbindable(elm, 'wheel', wheel, { passive: false }),
                            unbindable(document, 'mouseup', mouseup),
                            unbindable(document, 'touchend', mouseup),
                            unbindable(document, 'mousemove', mousemove, { passive: false }),
                            unbindable(document, 'touchmove', mousemove, { passive: false }),
                        ],
                    };
                },
                update(_, vnode) {
                    render(vnode.elm, ctrl, opts);
                },
                destroy(vnode) {
                    var _a;
                    const unbinds = (_a = vnode.elm.massim) === null || _a === void 0 ? void 0 : _a.unbinds;
                    if (unbinds)
                        for (const unbind of unbinds)
                            unbind();
                },
            },
        });
    }
    function unbindable(el, eventName, callback, options) {
        el.addEventListener(eventName, callback, options);
        return () => el.removeEventListener(eventName, callback, options);
    }
    function eventZoom(e) {
        var _a;
        if (((_a = e.targetTouches) === null || _a === void 0 ? void 0 : _a.length) !== 2)
            return;
        return {
            center: [
                (e.targetTouches[0].clientX + e.targetTouches[1].clientX) / 2,
                (e.targetTouches[0].clientY + e.targetTouches[1].clientY) / 2
            ],
            distance: Math.max(20, Math.hypot(e.targetTouches[0].clientX - e.targetTouches[1].clientX, e.targetTouches[0].clientY - e.targetTouches[1].clientY))
        };
    }
    function eventPosition(e) {
        var _a;
        if (e.clientX || e.clientX === 0)
            return [e.clientX, e.clientY];
        if ((_a = e.targetTouches) === null || _a === void 0 ? void 0 : _a[0])
            return [e.targetTouches[0].clientX, e.targetTouches[0].clientY];
        return;
    }
    function distanceSq(a, b) {
        const dx = a[0] - b[0];
        const dy = a[1] - b[1];
        return dx * dx + dy * dy;
    }
    function mod(a, b) {
        return ((a % b) + b) % b;
    }
    function render(canvas, ctrl, opts, raf = false) {
        const vm = ctrl.vm;
        const width = canvas.width, height = canvas.height;
        const ctx = canvas.getContext('2d');
        ctx.save();
        // font
        ctx.textBaseline = 'middle';
        ctx.textAlign = 'center';
        ctx.font = '0.4px Arial';
        // fill background
        ctx.fillStyle = '#eee';
        ctx.fillRect(0, 0, width, height);
        // draw grid
        const transform = ctrl.vm.transform;
        const selectedAgent = ctrl.selectedAgent();
        if ((opts === null || opts === void 0 ? void 0 : opts.viewOnly) && selectedAgent) {
            // auto center to selection
            transform.scale = Math.min(canvas.width, canvas.height) / (selectedAgent.vision * 2 + 3);
            transform.x = canvas.width / 2 - (selectedAgent.x + 0.5) * transform.scale;
            transform.y = canvas.height / 2 - (selectedAgent.y + 0.5) * transform.scale;
        }
        ctx.translate(transform.x, transform.y);
        ctx.scale(transform.scale, transform.scale);
        const ymin = Math.floor(-transform.y / transform.scale);
        const xmin = Math.floor(-transform.x / transform.scale);
        const ymax = ymin + Math.ceil(canvas.height / transform.scale);
        const xmax = xmin + Math.ceil(canvas.width / transform.scale);
        ctx.fillStyle = '#ddd';
        for (let y = ymin; y <= ymax; y++) {
            for (let x = xmin + (((xmin + y) % 2) + 2) % 2; x <= xmax; x += 2) {
                ctx.fillRect(x, y, 1, 1);
            }
        }
        if (ctrl.root.vm.static && ctrl.root.vm.dynamic) {
            const grid = ctrl.root.vm.static.grid;
            // terrain
            for (let y = ymin; y <= ymax; y++) {
                for (let x = xmin; x <= xmax; x++) {
                    switch (ctrl.root.vm.dynamic.cells[mod(y, grid.height)][mod(x, grid.width)]) {
                        case 1: // GOAL
                            ctx.fillStyle = goal;
                            ctx.fillRect(x, y, 1, 1);
                            break;
                        case 2: // OBSTABLE
                            ctx.fillStyle = obstacle;
                            ctx.fillRect(x - 0.04, y - 0.04, 1.08, 1.08);
                            break;
                    }
                }
            }
            for (let dy = Math.floor(ymin / grid.height) * grid.height; dy <= ymax + grid.height; dy += grid.height) {
                for (let dx = Math.floor(xmin / grid.width) * grid.width; dx <= xmax + grid.width; dx += grid.width) {
                    // draw axis
                    ctx.globalCompositeOperation = 'difference';
                    ctx.strokeStyle = 'white';
                    ctx.lineWidth = 0.3;
                    ctx.beginPath();
                    if (ctrl.root.vm.dynamic.taskboards) {
                        ctx.moveTo(dx - 1.5, dy);
                        ctx.lineTo(dx + 1.5, dy);
                        ctx.moveTo(dx, dy - 1.5);
                        ctx.lineTo(dx, dy + 1.5);
                    }
                    else {
                        // 2019
                        ctx.moveTo(dx, dy);
                        ctx.lineTo(dx + grid.width, dy);
                        ctx.moveTo(dx, dy);
                        ctx.lineTo(dx, dy + grid.height);
                    }
                    ctx.stroke();
                    ctx.globalCompositeOperation = 'source-over';
                    // dispensers
                    for (const dispenser of ctrl.root.vm.dynamic.dispensers) {
                        if (visible(xmin, xmax, ymin, ymax, dispenser, dx, dy)) {
                            ctx.lineWidth = 2 * 0.025;
                            const color = blocks[ctrl.root.vm.static.blockTypes.indexOf(dispenser.type) % blocks.length];
                            const r1 = rect(1, dx + dispenser.x, dy + dispenser.y, 0.025);
                            drawBlock(ctx, r1, color, 'white', 'black');
                            const r2 = rect(1, dx + dispenser.x, dy + dispenser.y, 5 * 0.025);
                            drawBlock(ctx, r2, color, 'white', 'black');
                            const r3 = rect(1, dx + dispenser.x, dy + dispenser.y, 9 * 0.025);
                            drawBlock(ctx, r3, color, 'white', 'black');
                            ctx.fillStyle = 'white';
                            ctx.fillText(dispenser.type, dx + dispenser.x + 0.5, dy + dispenser.y + 0.5);
                        }
                    }
                    // task boards
                    if (ctrl.root.vm.dynamic.taskboards) {
                        for (const board$1 of ctrl.root.vm.dynamic.taskboards) {
                            if (visible(xmin, xmax, ymin, ymax, board$1, dx, dy)) {
                                ctx.lineWidth = 0.05;
                                drawBlock(ctx, rect(1, dx + board$1.x, dy + board$1.y, 0.05), board, 'white', 'black');
                            }
                        }
                    }
                    // blocks
                    drawBlocks(ctx, dx, dy, ctrl.root.vm.static, ctrl.root.vm.dynamic.blocks.filter(b => visible(xmin, xmax, ymin, ymax, b, dx, dy)));
                    // agents
                    for (const agent of ctrl.root.vm.dynamic.entities) {
                        if (visible(xmin, xmax, ymin, ymax, agent, dx, dy)) {
                            const teamIndex = ctrl.root.vm.teamNames.indexOf(agent.team);
                            drawAgent(ctx, dx, dy, agent, teamIndex);
                        }
                        // agent action
                        if (agent.action == 'clear' && agent.actionResult.indexOf('failed_') != 0) {
                            const x = dx + agent.x + parseInt(agent.actionParams[0], 10);
                            const y = dy + agent.y + parseInt(agent.actionParams[1], 10);
                            ctx.lineWidth = 0.05;
                            ctx.strokeStyle = 'red';
                            drawArea(ctx, x, y, 1);
                        }
                    }
                    // attachables of selected agent
                    if (selectedAgent === null || selectedAgent === void 0 ? void 0 : selectedAgent.attached) {
                        ctx.fillStyle = hover;
                        for (const attached of selectedAgent.attached) {
                            if (attached.x != selectedAgent.x || attached.y != selectedAgent.y) {
                                ctx.fillRect(dx + attached.x, dy + attached.y, 1, 1);
                            }
                        }
                    }
                    // clear events
                    for (const clear of ctrl.root.vm.dynamic.clear) {
                        ctx.lineWidth = 0.1;
                        ctx.strokeStyle = 'red';
                        drawArea(ctx, dx + clear.x, dy + clear.y, clear.radius);
                    }
                    // hover
                    if (ctrl.root.vm.hover) {
                        drawHover(ctx, ctrl.root.vm.static, ctrl.root.vm.dynamic, ctrl.root.vm.teamNames, dx, dy, ctrl.root.vm.hover);
                    }
                }
            }
            // fog of war
            for (let dy = Math.floor(ymin / grid.height) * grid.height; dy <= ymax + grid.height; dy += grid.height) {
                for (let dx = Math.floor(xmin / grid.width) * grid.width; dx <= xmax + grid.width; dx += grid.width) {
                    for (const agent of ctrl.root.vm.dynamic.entities) {
                        if (agent.id === ctrl.vm.selected) {
                            drawFogOfWar(ctx, ctrl.root.vm.static, dx, dy, agent);
                        }
                    }
                }
            }
        }
        ctx.restore();
        if (raf && (vm.dragging || vm.zooming)) {
            requestAnimationFrame(() => render(canvas, ctrl, opts, true));
        }
    }
    function visible(xmin, xmax, ymin, ymax, pos, dx, dy) {
        return xmin <= pos.x + dx && pos.x + dx <= xmax && ymin <= pos.y + dy && pos.y + dy <= ymax;
    }
    function drawFogOfWar(ctx, st, dx, dy, agent) {
        ctx.fillStyle = 'rgba(0, 0, 0, 0.3)';
        const top = dy - st.grid.height + agent.y + agent.vision + 1;
        ctx.fillRect(dx, top, st.grid.width, st.grid.height - 2 * agent.vision - 1); // above
        ctx.fillRect(dx - st.grid.width + agent.x + agent.vision + 1, dy + agent.y - agent.vision, st.grid.width - 2 * agent.vision - 1, 2 * agent.vision + 1);
        for (let x = -agent.vision; x <= agent.vision; x++) {
            for (let y = -agent.vision; y <= agent.vision; y++) {
                if (Math.abs(x) + Math.abs(y) > agent.vision) {
                    ctx.fillRect(dx + agent.x + x, dy + agent.y + y, 1, 1);
                }
            }
        }
    }
    function drawHover(ctx, st, world, teamNames, dx, dy, hover$1) {
        if (hover$1.x < 0 || hover$1.x >= st.grid.width || hover$1.y < 0 || hover$1.y >= st.grid.height)
            return;
        ctx.beginPath();
        ctx.fillStyle = hover;
        ctx.fillRect(dx + hover$1.x, dy + hover$1.y, 1, 1);
        for (const attachable of world.entities.concat(world.blocks)) {
            if (attachable.x == hover$1.x && attachable.y == hover$1.y && attachable.attached) {
                for (let pos of attachable.attached) {
                    ctx.fillRect(dx + pos.x, dy + pos.y, 1, 1);
                }
            }
        }
        ctx.lineWidth = 0.1;
        if (world.taskboards)
            for (const taskboard of world.taskboards) {
                if (Math.abs(taskboard.x - hover$1.x) + Math.abs(taskboard.y - hover$1.y) <= 2) {
                    ctx.strokeStyle = board;
                    drawArea(ctx, dx + taskboard.x, dy + taskboard.y, 2);
                }
            }
        for (const agent of world.entities) {
            if (Math.abs(agent.x - hover$1.x) + Math.abs(agent.y - hover$1.y) <= agent.vision) {
                ctx.strokeStyle = teams[teamNames.indexOf(agent.team)];
                drawArea(ctx, dx + agent.x, dy + agent.y, 5);
            }
        }
    }
    function rect(blockSize, x, y, margin) {
        return {
            x1: x * blockSize + margin,
            y1: y * blockSize + margin,
            x2: x * blockSize + blockSize - margin,
            y2: y * blockSize + blockSize - margin,
            width: blockSize - 2 * margin,
            height: blockSize - 2 * margin,
        };
    }
    function drawAgent(ctx, dx, dy, agent, teamIndex) {
        ctx.lineWidth = 0.125;
        ctx.strokeStyle = 'black';
        ctx.beginPath();
        ctx.moveTo(dx + agent.x + 0.5, dy + agent.y);
        ctx.lineTo(dx + agent.x + 0.5, dy + agent.y + 1);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(dx + agent.x, dy + agent.y + 0.5);
        ctx.lineTo(dx + agent.x + 1, dy + agent.y + 0.5);
        ctx.stroke();
        const color = teams[teamIndex];
        if (teamIndex === 0) {
            ctx.lineWidth = 0.05;
            const margin = (1 - 15 / 16 / Math.sqrt(2)) / 2;
            const r = rect(1, dx + agent.x, dy + agent.y, margin);
            drawBlock(ctx, r, color, 'white', 'black');
        }
        else {
            ctx.lineWidth = 0.04;
            const r = rect(1, dx + agent.x, dy + agent.y, 0.0625);
            drawRotatedBlock(ctx, r, color, 'white', 'black');
        }
        if (agent.name) {
            ctx.fillStyle = 'white';
            ctx.fillText(shortAgentName(agent.name), dx + agent.x + 0.5, dy + agent.y + 0.5);
        }
    }
    function drawBlocks(ctx, dx, dy, st, blocks$1) {
        for (const block of blocks$1) {
            ctx.lineWidth = 0.05;
            const r = rect(1, dx + block.x, dy + block.y, 0.025);
            drawBlock(ctx, r, blocks[st.blockTypes.indexOf(block.type) % blocks.length], 'white', 'black');
            ctx.textBaseline = 'middle';
            ctx.textAlign = 'center';
            ctx.fillStyle = 'white';
            ctx.font = '0.5px Arial';
            ctx.fillText(block.type, dx + block.x + 0.5, dy + block.y + 0.5);
        }
    }
    function drawBlock(ctx, r, color, light, dark) {
        ctx.fillStyle = color;
        ctx.fillRect(r.x1, r.y1, r.width, r.height);
        ctx.beginPath();
        ctx.moveTo(r.x1, r.y2);
        ctx.lineTo(r.x1, r.y1);
        ctx.lineTo(r.x2, r.y1);
        ctx.strokeStyle = light;
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(r.x2, r.y1);
        ctx.lineTo(r.x2, r.y2);
        ctx.lineTo(r.x1, r.y2);
        ctx.strokeStyle = dark;
        ctx.stroke();
    }
    function drawArea(ctx, x, y, radius) {
        ctx.beginPath();
        ctx.moveTo(x - radius, y + 0.5);
        ctx.lineTo(x + 0.5, y - radius);
        ctx.lineTo(x + 1 + radius, y + 0.5);
        ctx.lineTo(x + 0.5, y + radius + 1);
        ctx.lineTo(x - radius, y + 0.5);
        ctx.stroke();
    }
    function drawRotatedBlock(ctx, r, color, light, dark) {
        ctx.beginPath();
        ctx.fillStyle = color;
        ctx.moveTo(r.x1, (r.y1 + r.y2) / 2);
        ctx.lineTo((r.x1 + r.x2) / 2, r.y1);
        ctx.lineTo(r.x2, (r.y1 + r.y2) / 2);
        ctx.lineTo((r.x1 + r.x2) / 2, r.y2);
        ctx.closePath();
        ctx.fill();
        ctx.beginPath();
        ctx.moveTo(r.x1, (r.y1 + r.y2) / 2);
        ctx.lineTo((r.x1 + r.x2) / 2, r.y1);
        ctx.lineTo(r.x2, (r.y1 + r.y2) / 2);
        ctx.strokeStyle = light;
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(r.x2, (r.y1 + r.y2) / 2);
        ctx.lineTo((r.x1 + r.x2) / 2, r.y2);
        ctx.lineTo(r.x1, (r.y1 + r.y2) / 2);
        ctx.strokeStyle = dark;
        ctx.stroke();
    }
    function shortAgentName(name) {
        const match = name.match(/^agent-?[A-Za-z][A-Za-z-_]*([0-9]+)$/);
        return match ? match[1] : name;
    }

    class Ctrl {
        constructor(redraw, replayPath) {
            this.redraw = redraw;
            this.vm = {
                state: 'connecting',
                teamNames: [],
            };
            if (replayPath)
                this.replay = new ReplayCtrl(this, replayPath);
            else
                this.connect();
            this.map = new MapCtrl(this);
            this.maps = [];
        }
        connect() {
            const protocol = document.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const path = document.location.pathname.substr(0, document.location.pathname.lastIndexOf('/'));
            const ws = new WebSocket(protocol + '//' + document.location.host + path + '/live/monitor');
            ws.onmessage = (msg) => {
                const data = JSON.parse(msg.data);
                console.log(data);
                if (data.grid)
                    this.setStatic(data);
                else
                    this.setDynamic(data);
                this.redraw();
            };
            ws.onopen = () => {
                console.log('Connected');
                this.vm.state = 'online';
                this.redraw();
            };
            ws.onclose = () => {
                console.log('Disconnected');
                setTimeout(() => this.connect(), 5000);
                this.vm.state = 'offline';
                this.redraw();
            };
        }
        setStatic(st) {
            if (st) {
                st.blockTypes.sort();
                this.vm.teamNames = Object.keys(st.teams);
                this.vm.teamNames.sort();
            }
            this.vm.static = st;
            this.resetTransform();
        }
        resetTransform() {
            var _a;
            const margin = 2;
            const grid = (_a = this.vm.static) === null || _a === void 0 ? void 0 : _a.grid;
            if (!grid)
                return;
            const scale = Math.max(minScale, Math.min(maxScale, Math.min(window.innerWidth, window.innerHeight) / (2 * margin + Math.max(grid.width, grid.height))));
            this.map.vm.transform = {
                x: (window.innerWidth - scale * (grid.width + 2 * margin)) / 2 + scale * margin,
                y: (window.innerHeight - scale * (grid.height + 2 * margin)) / 2 + scale * margin,
                scale,
            };
        }
        setDynamic(dynamic) {
            if (dynamic)
                dynamic.entities.sort(compareAgent);
            this.vm.dynamic = dynamic;
        }
        toggleMaps() {
            if (this.vm.dynamic && !this.maps.length) {
                this.maps = this.vm.dynamic.entities.map(agent => {
                    const ctrl = new MapCtrl(this);
                    ctrl.vm.selected = agent.id;
                    return ctrl;
                });
            }
            else {
                this.maps = [];
            }
            this.redraw();
        }
        setHover(pos) {
            const changed = (!pos && this.vm.hover) || (pos && !this.vm.hover) || (pos && this.vm.hover && (pos.x != this.vm.hover.x || pos.y != this.vm.hover.y));
            this.vm.hover = pos;
            if (changed)
                this.redraw();
        }
    }
    class ReplayCtrl {
        constructor(root, path) {
            this.root = root;
            this.path = path;
            this.step = -1;
            this.cache = new Map();
            if (path[path.length - 1] == '/')
                path = path.substr(0, path.length - 1);
            this.suffix = location.pathname == '/' ? `?sri=${Math.random().toString(36).slice(-8)}` : '';
            this.loadStatic();
        }
        stop() {
            if (this.timer)
                clearInterval(this.timer);
            this.timer = undefined;
            this.root.redraw();
        }
        start() {
            if (!this.timer)
                this.timer = setInterval(() => {
                    if (this.root.vm.state !== 'connecting')
                        this.setStep(this.step + 1);
                }, 1000);
            this.root.redraw();
        }
        loadStatic() {
            const xhr = new XMLHttpRequest();
            xhr.open('GET', `${this.path}/static.json${this.suffix}`);
            xhr.onload = () => {
                if (xhr.status === 200) {
                    this.root.setStatic(JSON.parse(xhr.responseText));
                    this.setStep(this.step);
                }
                else {
                    this.root.vm.state = 'error';
                }
                this.root.redraw();
            };
            xhr.onerror = () => {
                this.root.vm.state = 'error';
                this.root.redraw();
            };
            xhr.send();
        }
        loadDynamic(step) {
            // got from cache
            const entry = this.cache.get(step);
            if (entry) {
                this.root.setDynamic(entry);
                this.root.vm.state = (this.root.vm.dynamic && this.root.vm.dynamic.step == step) ? 'online' : 'connecting';
                this.root.redraw();
                return;
            }
            const onerror = () => {
                this.root.vm.state = 'error';
                this.stop();
                this.root.redraw();
            };
            const group = step > 0 ? Math.floor(step / 5) * 5 : 0;
            const xhr = new XMLHttpRequest();
            xhr.open('GET', `${this.path}/${group}.json${this.suffix}`);
            xhr.onload = () => {
                if (xhr.status === 200) {
                    const response = JSON.parse(xhr.responseText);
                    // write to cache
                    if (this.cache.size > 100)
                        this.cache.clear();
                    for (const s in response)
                        this.cache.set(parseInt(s, 10), response[s]);
                    if (response[step]) {
                        this.root.setDynamic(response[step]);
                        this.root.vm.state = (this.root.vm.dynamic && this.root.vm.dynamic.step == step) ? 'online' : 'connecting';
                        this.root.redraw();
                        return;
                    }
                }
                // status !== 200 or !response[step]
                onerror();
            };
            xhr.onerror = onerror;
            xhr.send();
        }
        setStep(s) {
            // keep step in bounds
            this.step = Math.max(-1, s);
            if (this.root.vm.static && this.step >= this.root.vm.static.steps) {
                this.stop();
                this.step = this.root.vm.static.steps - 1;
            }
            // show connecting after a while
            this.root.vm.state = 'connecting';
            setTimeout(() => this.root.redraw(), 500);
            // update url
            if (history.replaceState)
                history.replaceState({}, document.title, '#' + this.step);
            this.loadDynamic(this.step);
        }
        name() {
            const parts = this.path.split('/');
            return parts[parts.length - 1];
        }
        toggle() {
            if (this.timer)
                this.stop();
            else
                this.start();
        }
        playing() {
            return !!this.timer;
        }
    }

    var vnode_1 = createCommonjsModule(function (module, exports) {
    Object.defineProperty(exports, "__esModule", { value: true });
    function vnode(sel, data, children, text, elm) {
        var key = data === undefined ? undefined : data.key;
        return { sel: sel, data: data, children: children, text: text, elm: elm, key: key };
    }
    exports.vnode = vnode;
    exports.default = vnode;

    });

    var is = createCommonjsModule(function (module, exports) {
    Object.defineProperty(exports, "__esModule", { value: true });
    exports.array = Array.isArray;
    function primitive(s) {
        return typeof s === 'string' || typeof s === 'number';
    }
    exports.primitive = primitive;

    });

    var h_1 = createCommonjsModule(function (module, exports) {
    Object.defineProperty(exports, "__esModule", { value: true });


    function addNS(data, children, sel) {
        data.ns = 'http://www.w3.org/2000/svg';
        if (sel !== 'foreignObject' && children !== undefined) {
            for (var i = 0; i < children.length; ++i) {
                var childData = children[i].data;
                if (childData !== undefined) {
                    addNS(childData, children[i].children, children[i].sel);
                }
            }
        }
    }
    function h(sel, b, c) {
        var data = {}, children, text, i;
        if (c !== undefined) {
            data = b;
            if (is.array(c)) {
                children = c;
            }
            else if (is.primitive(c)) {
                text = c;
            }
            else if (c && c.sel) {
                children = [c];
            }
        }
        else if (b !== undefined) {
            if (is.array(b)) {
                children = b;
            }
            else if (is.primitive(b)) {
                text = b;
            }
            else if (b && b.sel) {
                children = [b];
            }
            else {
                data = b;
            }
        }
        if (children !== undefined) {
            for (i = 0; i < children.length; ++i) {
                if (is.primitive(children[i]))
                    children[i] = vnode_1.vnode(undefined, undefined, undefined, children[i], undefined);
            }
        }
        if (sel[0] === 's' && sel[1] === 'v' && sel[2] === 'g' &&
            (sel.length === 3 || sel[3] === '.' || sel[3] === '#')) {
            addNS(data, children, sel);
        }
        return vnode_1.vnode(sel, data, children, text, undefined);
    }
    exports.h = h;
    exports.default = h;

    });

    function replay(ctrl) {
        return h('div.box.replay', [
            h('div', [h('strong', 'Replay:'), ' ', ctrl.name()]),
            h('div', [
                h('button', { on: { click: () => ctrl.setStep(-1) } }, '|<<'),
                h('button', { on: { click: () => ctrl.setStep(ctrl.step - 10) } }, '<<'),
                h('button', {
                    on: { click: () => ctrl.toggle() }
                }, ctrl.playing() ? '||' : '>'),
                h('button', { on: { click: () => ctrl.setStep(ctrl.step + 10) } }, '>>'),
                h('button', { on: { click: () => ctrl.setStep(99999999) } }, '>>|')
            ])
        ]);
    }
    function simplePlural(n, singular) {
        if (n === 1)
            return '1 ' + singular;
        else
            return n + ' ' + singular + 's';
    }
    function teams$1(teamNames, world) {
        return teamNames.map((name, i) => h('div.team', {
            style: { background: teams[i] }
        }, `${name}: $${world.scores[name]}`));
    }
    function tasks(ctrl, st, world) {
        const selectedTask = world.tasks.find(t => t.name === ctrl.vm.taskName);
        return [
            h('select', {
                props: {
                    value: ctrl.vm.taskName || '',
                },
                on: {
                    change: function (e) {
                        ctrl.vm.taskName = e.target.value;
                        ctrl.redraw();
                    }
                }
            }, [
                h('option', {
                    attrs: {
                        value: ''
                    },
                }, simplePlural(world.tasks.length, 'task')),
                ...world.tasks.map(t => {
                    const acceptedBy = world.entities.filter(a => a.acceptedTask === t.name).length;
                    return h('option', {
                        attrs: {
                            value: t.name
                        },
                    }, [
                        `$${t.reward} for ${t.name} until step ${t.deadline}`,
                        acceptedBy ? ` (${acceptedBy} accepted)` : undefined,
                    ]);
                }),
            ]),
            ...(selectedTask ? taskDetails(ctrl, st, world, selectedTask) : [])
        ];
    }
    function hover$1(ctrl, st, world, pos) {
        if (!world.cells[pos.y])
            return;
        const terrain = world.cells[pos.y][pos.x];
        if (typeof terrain == 'undefined')
            return;
        // pos
        const r = [h('li', `x = ${pos.x}, y = ${pos.y}`)];
        // terrain
        if (terrain === 1)
            r.push(h('li', ['terrain: ', h('span', {
                    style: {
                        background: goalOnLight,
                        color: 'black',
                    }
                }, 'goal')]));
        else if (terrain === 2)
            r.push(h('li', 'terrain: obstacle'));
        // dispensers
        for (const dispenser of world.dispensers) {
            if (dispenser.x == pos.x && dispenser.y == pos.y) {
                r.push(h('li', ['dispenser: type = ', blockSpan(st, dispenser.type)]));
            }
        }
        // task boards
        if (world.taskboards) {
            for (const board$1 of world.taskboards) {
                if (board$1.x == pos.x && board$1.y == pos.y) {
                    r.push(h('li', h('span', {
                        style: {
                            background: board,
                            color: 'black',
                        }
                    }, 'task board')));
                }
            }
        }
        // blocks
        for (const block of world.blocks) {
            if (block.x == pos.x && block.y == pos.y) {
                r.push(h('li', ['block: type = ', blockSpan(st, block.type)]));
            }
        }
        // agents
        for (const agent of world.entities) {
            if (agent.x == pos.x && agent.y == pos.y) {
                r.push(h('li', ['agent: ', ...agentDescription(ctrl, agent)]));
            }
        }
        return h('ul', r);
    }
    function blockSpan(st, type) {
        return h('span', {
            style: {
                background: blocks[st.blockTypes.indexOf(type)],
                color: 'white',
            }
        }, type);
    }
    function agentDescription(ctrl, agent) {
        var _a;
        const r = [
            'name = ', h('span', {
                style: {
                    background: teams[ctrl.vm.teamNames.indexOf(agent.team)],
                }
            }, agent.name),
            `, energy = ${agent.energy}`
        ];
        if (agent.action && agent.actionResult)
            r.push(', ', h('span', {
                class: {
                    [agent.action]: true,
                    [agent.actionResult]: true,
                }
            }, `${agent.action}(…) = ${agent.actionResult}`));
        if ((_a = agent.attached) === null || _a === void 0 ? void 0 : _a.length)
            r.push(`, ${agent.attached.length}\xa0attached`);
        if (agent.acceptedTask)
            r.push(', ', h('a', {
                on: {
                    click() {
                        ctrl.vm.taskName = agent.acceptedTask;
                        ctrl.redraw();
                    }
                }
            }, agent.acceptedTask));
        if (agent.disabled)
            r.push(', disabled');
        return r;
    }
    function taskDetails(ctrl, st, dynamic, task) {
        const xs = task.requirements.map(b => Math.abs(b.x));
        const ys = task.requirements.map(b => Math.abs(b.y));
        const width = 2 * Math.max(...xs) + 1;
        const height = 2 * Math.max(...ys) + 1;
        const elementWidth = 218;
        const gridSize = Math.min(Math.floor(elementWidth / width), 50);
        const elementHeight = gridSize * height;
        const render = function (vnode) {
            const canvas = vnode.elm;
            const ctx = canvas.getContext('2d');
            ctx.save();
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.translate((elementWidth - gridSize) / 2, (elementHeight - gridSize) / 2);
            ctx.scale(gridSize, gridSize);
            drawAgent(ctx, 0, 0, { x: 0, y: 0 }, 0);
            drawBlocks(ctx, 0, 0, st, task.requirements);
            ctx.restore();
        };
        const acceptedBy = dynamic.entities.filter(a => a.acceptedTask === task.name);
        return [
            h('canvas', {
                props: {
                    width: elementWidth,
                    height: elementHeight
                },
                hook: {
                    insert: render,
                    update: (_, vnode) => render(vnode)
                }
            }),
            ...(acceptedBy.length ? [
                h('p', `Accepted by ${simplePlural(acceptedBy.length, 'agent')}:`),
                h('ul', acceptedBy.map(by => h('li', h('a', {
                    style: {
                        background: teams[ctrl.vm.teamNames.indexOf(by.team)],
                    },
                    on: {
                        click() {
                            ctrl.map.vm.selected = by.id;
                            ctrl.redraw();
                        }
                    },
                }, by.name)))),
            ] : []),
            h('p', simplePlural(task.requirements.length, 'block')),
        ];
    }
    function disconnected() {
        return h('div.box', [
            h('p', 'Live server not connected.'),
            h('a', {
                props: { href: document.location.pathname + document.location.search }
            }, 'Retry now.')
        ]);
    }
    function box(child) {
        return child ? h('div.box', child) : undefined;
    }
    function overlay(ctrl) {
        const selectedAgent = ctrl.map.selectedAgent();
        return h('div#overlay', [
            ctrl.vm.static && (ctrl.replay ? replay(ctrl.replay) : h('div.box', ctrl.vm.static.sim)),
            (ctrl.vm.state === 'error' || ctrl.vm.state === 'offline') ?
                ctrl.replay ?
                    h('div.box', ctrl.vm.static ? 'Could not load step' : 'Could not load replay') :
                    disconnected() : undefined,
            (ctrl.vm.static && ctrl.vm.dynamic) ?
                h('div.box', [
                    `Step: ${ctrl.vm.dynamic.step} / ${ctrl.vm.static.steps - 1}`
                ]) : undefined,
            ctrl.vm.state === 'connecting' ? h('div.box', ['Connecting ...', h('div.loader')]) : undefined,
            (ctrl.vm.state === 'online' && (!ctrl.vm.static || !ctrl.vm.dynamic)) ? h('div.box', ['Waiting ...', h('div.loader')]) : undefined,
            ...((ctrl.vm.state === 'online' && ctrl.vm.static && ctrl.vm.dynamic) ? [
                h('div.box', teams$1(ctrl.vm.teamNames, ctrl.vm.dynamic)),
                h('div.box', [
                    h('button', {
                        on: {
                            click: () => ctrl.toggleMaps(),
                        }
                    }, ctrl.maps.length ? 'Global view' : 'Agent view'),
                    ctrl.maps.length ? undefined : h('button', {
                        on: {
                            click() {
                                ctrl.resetTransform();
                                ctrl.redraw();
                            }
                        }
                    }, 'Reset zoom'),
                ]),
                h('div.box', tasks(ctrl, ctrl.vm.static, ctrl.vm.dynamic)),
                selectedAgent ? box(h('div', ['Selected agent: ', ...agentDescription(ctrl, selectedAgent)])) : undefined,
                ctrl.vm.hover ? box(hover$1(ctrl, ctrl.vm.static, ctrl.vm.dynamic, ctrl.vm.hover)) : undefined,
            ] : [])
        ]);
    }

    function view(ctrl) {
        return h_1.h('div#monitor', [
            ctrl.maps.length ? agentView(ctrl) : mapView(ctrl.map),
            overlay(ctrl),
        ]);
    }
    function agentView(ctrl) {
        if (!ctrl.vm.static)
            return;
        return h_1.h('div.maps', ctrl.maps.map(m => {
            const agent = m.selectedAgent();
            if (!agent)
                return;
            const acceptedTask = agent.acceptedTask;
            return h_1.h('div', {
                class: (agent.action && agent.actionResult) ? {
                    'map': true,
                    [agent.action]: true,
                    [agent.actionResult]: true,
                } : {
                    map: true,
                },
            }, [
                h_1.h('a.team', {
                    style: m.vm.selected === ctrl.map.vm.selected ? {
                        background: 'white',
                        color: 'black',
                    } : {
                        background: teams[ctrl.vm.teamNames.indexOf(agent.team)],
                    },
                    on: {
                        click() {
                            ctrl.map.vm.selected = agent.id;
                            ctrl.toggleMaps();
                        },
                    },
                }, `${agent.name} (${agent.x}|${agent.y})`),
                mapView(m, {
                    size: 250,
                    viewOnly: true,
                }),
                h_1.h('div.meta', [
                    h_1.h('div', `energy = ${agent.energy}`),
                    agent.action ? h_1.h('div', `${agent.action}(…) = ${agent.actionResult}`) : undefined,
                    acceptedTask ? h_1.h('a', {
                        on: {
                            click() {
                                ctrl.vm.taskName = acceptedTask;
                                ctrl.redraw();
                            }
                        }
                    }, agent.acceptedTask) : undefined,
                    agent.disabled ? h_1.h('div', 'disabled') : undefined,
                ]),
            ]);
        }));
    }

    function makeStatusCtrl(redraw) {
        const vm = {
            state: 'connecting'
        };
        function connect() {
            const protocol = document.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const path = document.location.pathname.substr(0, document.location.pathname.lastIndexOf('/'));
            const ws = new WebSocket(protocol + '//' + document.location.host + path + '/live/status');
            ws.onmessage = (msg) => {
                const data = JSON.parse(msg.data);
                console.log(data);
                vm.data = data;
                redraw();
            };
            ws.onopen = () => {
                console.log('Connected');
                vm.state = 'online';
                redraw();
            };
            ws.onclose = () => {
                console.log('Disconnected');
                setTimeout(() => connect(), 5000);
                vm.data = undefined;
                vm.state = 'offline';
                redraw();
            };
        }
        connect();
        return {
            vm,
            redraw
        };
    }

    function view$1(data) {
        data.entities.sort(compareAgent);
        const teams$1 = [];
        for (const entity of data.entities) {
            if (teams$1.indexOf(entity.team) == -1)
                teams$1.push(entity.team);
        }
        return [
            h('h2', `Step ${data.step}/${data.steps - 1}`),
            h('table', [
                h('thead', [
                    h('tr', [
                        h('th', 'Team'),
                        h('th', 'Agent'),
                        h('th', 'Last action'),
                        h('th', 'Last action result')
                    ])
                ]),
                h('tbody', data.entities.map((entity) => {
                    const teamColors = { style: { background: teams[teams$1.indexOf(entity.team)] } };
                    return h('tr', [
                        h('td', teamColors, entity.team),
                        h('td', teamColors, entity.name),
                        h('td', { attrs: { class: entity.action } }, entity.action),
                        h('td', { attrs: { class: entity.actionResult } }, entity.actionResult)
                    ]);
                }))
            ])
        ];
    }
    function statusView(ctrl) {
        return h('div#status', [
            h('h1', ['Status: ', ctrl.vm.data ? ctrl.vm.data.sim : ctrl.vm.state]),
            ...(ctrl.vm.data ? view$1(ctrl.vm.data) : [])
        ]);
    }

    const patch = init([
        _class.classModule,
        props.propsModule,
        attributes.attributesModule,
        style.styleModule,
        eventlisteners.eventListenersModule
    ]);
    function Monitor(element) {
        let vnode = element;
        let ctrl;
        let redrawRequested = false;
        const redraw = function () {
            if (redrawRequested)
                return;
            redrawRequested = true;
            requestAnimationFrame(() => {
                redrawRequested = false;
                vnode = patch(vnode, view(ctrl));
            });
        };
        const hashChange = function () {
            if (ctrl.replay) {
                const step = parseInt(document.location.hash.substr(1), 10);
                if (step > 0)
                    ctrl.replay.setStep(step);
                else if (!document.location.hash)
                    ctrl.replay.start();
            }
        };
        const replayPath = window.location.search.length > 1 ? window.location.search.substr(1) : undefined;
        ctrl = new Ctrl(redraw, replayPath);
        hashChange();
        window.onhashchange = hashChange;
        redraw();
        /* canvas.addEventListener('mousemove', e => {
          if (!ctrl.vm.static) return;
          ctrl.setHover(invClientPos(canvas, ctrl.vm.static, e.clientX, e.clientY));
        });
        canvas.addEventListener('mouseleave', e => {
          ctrl.setHover(undefined);
        }); */
    }
    function Status(target) {
        let vnode = target;
        let ctrl;
        let redrawRequested = false;
        const redraw = function () {
            if (redrawRequested)
                return;
            redrawRequested = true;
            requestAnimationFrame(() => {
                redrawRequested = false;
                vnode = patch(vnode, statusView(ctrl));
            });
        };
        ctrl = makeStatusCtrl(redraw);
        redraw();
    }

    exports.Monitor = Monitor;
    exports.Status = Status;

    return exports;

}({}));
