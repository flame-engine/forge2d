(function dartProgram(){function copyProperties(a,b){var s=Object.keys(a)
for(var r=0;r<s.length;r++){var q=s[r]
b[q]=a[q]}}function mixinPropertiesHard(a,b){var s=Object.keys(a)
for(var r=0;r<s.length;r++){var q=s[r]
if(!b.hasOwnProperty(q)){b[q]=a[q]}}}function mixinPropertiesEasy(a,b){Object.assign(b,a)}var z=function(){var s=function(){}
s.prototype={p:{}}
var r=new s()
if(!(Object.getPrototypeOf(r)&&Object.getPrototypeOf(r).p===s.prototype.p))return false
try{if(typeof navigator!="undefined"&&typeof navigator.userAgent=="string"&&navigator.userAgent.indexOf("Chrome/")>=0)return true
if(typeof version=="function"&&version.length==0){var q=version()
if(/^\d+\.\d+\.\d+\.\d+$/.test(q))return true}}catch(p){}return false}()
function inherit(a,b){a.prototype.constructor=a
a.prototype["$i"+a.name]=a
if(b!=null){if(z){Object.setPrototypeOf(a.prototype,b.prototype)
return}var s=Object.create(b.prototype)
copyProperties(a.prototype,s)
a.prototype=s}}function inheritMany(a,b){for(var s=0;s<b.length;s++){inherit(b[s],a)}}function mixinEasy(a,b){mixinPropertiesEasy(b.prototype,a.prototype)
a.prototype.constructor=a}function mixinHard(a,b){mixinPropertiesHard(b.prototype,a.prototype)
a.prototype.constructor=a}function lazy(a,b,c,d){var s=a
a[b]=s
a[c]=function(){if(a[b]===s){a[b]=d()}a[c]=function(){return this[b]}
return a[b]}}function lazyFinal(a,b,c,d){var s=a
a[b]=s
a[c]=function(){if(a[b]===s){var r=d()
if(a[b]!==s){A.i4(b)}a[b]=r}var q=a[b]
a[c]=function(){return q}
return q}}function makeConstList(a,b){if(b!=null)A.h(a,b)
a.$flags=7
return a}function convertToFastObject(a){function t(){}t.prototype=a
new t()
return a}function convertAllToFastObject(a){for(var s=0;s<a.length;++s){convertToFastObject(a[s])}}var y=0
function instanceTearOffGetter(a,b){var s=null
return a?function(c){if(s===null)s=A.fI(b)
return new s(c,this)}:function(){if(s===null)s=A.fI(b)
return new s(this,null)}}function staticTearOffGetter(a){var s=null
return function(){if(s===null)s=A.fI(a).prototype
return s}}var x=0
function tearOffParameters(a,b,c,d,e,f,g,h,i,j){if(typeof h=="number"){h+=x}return{co:a,iS:b,iI:c,rC:d,dV:e,cs:f,fs:g,fT:h,aI:i||0,nDA:j}}function installStaticTearOff(a,b,c,d,e,f,g,h){var s=tearOffParameters(a,true,false,c,d,e,f,g,h,false)
var r=staticTearOffGetter(s)
a[b]=r}function installInstanceTearOff(a,b,c,d,e,f,g,h,i,j){c=!!c
var s=tearOffParameters(a,false,c,d,e,f,g,h,i,!!j)
var r=instanceTearOffGetter(c,s)
a[b]=r}function setOrUpdateInterceptorsByTag(a){var s=v.interceptorsByTag
if(!s){v.interceptorsByTag=a
return}copyProperties(a,s)}function setOrUpdateLeafTags(a){var s=v.leafTags
if(!s){v.leafTags=a
return}copyProperties(a,s)}function updateTypes(a){var s=v.types
var r=s.length
s.push.apply(s,a)
return r}function updateHolder(a,b){copyProperties(b,a)
return a}var hunkHelpers=function(){var s=function(a,b,c,d,e){return function(f,g,h,i){return installInstanceTearOff(f,g,a,b,c,d,[h],i,e,false)}},r=function(a,b,c,d){return function(e,f,g,h){return installStaticTearOff(e,f,a,b,c,[g],h,d)}}
return{inherit:inherit,inheritMany:inheritMany,mixin:mixinEasy,mixinHard:mixinHard,installStaticTearOff:installStaticTearOff,installInstanceTearOff:installInstanceTearOff,_instance_0u:s(0,0,null,["$0"],0),_instance_1u:s(0,1,null,["$1"],0),_instance_2u:s(0,2,null,["$2"],0),_instance_0i:s(1,0,null,["$0"],0),_instance_1i:s(1,1,null,["$1"],0),_instance_2i:s(1,2,null,["$2"],0),_static_0:r(0,null,["$0"],0),_static_1:r(1,null,["$1"],0),_static_2:r(2,null,["$2"],0),makeConstList:makeConstList,lazy:lazy,lazyFinal:lazyFinal,updateHolder:updateHolder,convertToFastObject:convertToFastObject,updateTypes:updateTypes,setOrUpdateInterceptorsByTag:setOrUpdateInterceptorsByTag,setOrUpdateLeafTags:setOrUpdateLeafTags}}()
function initializeDeferredHunk(a){x=v.types.length
a(hunkHelpers,v,w,$)}var J={
fL(a,b,c,d){return{i:a,p:b,e:c,x:d}},
fJ(a){var s,r,q,p,o,n=a[v.dispatchPropertyName]
if(n==null)if($.fK==null){A.kD()
n=a[v.dispatchPropertyName]}if(n!=null){s=n.p
if(!1===s)return n.i
if(!0===s)return a
r=Object.getPrototypeOf(a)
if(s===r)return n.i
if(n.e===r)throw A.i(A.hc("Return interceptor for "+A.r(s(a,n))))}q=a.constructor
if(q==null)p=null
else{o=$.eE
if(o==null)o=$.eE=v.getIsolateTag("_$dart_js")
p=q[o]}if(p!=null)return p
p=A.kI(a)
if(p!=null)return p
if(typeof a=="function")return B.Q
s=Object.getPrototypeOf(a)
if(s==null)return B.z
if(s===Object.prototype)return B.z
if(typeof q=="function"){o=$.eE
if(o==null)o=$.eE=v.getIsolateTag("_$dart_js")
Object.defineProperty(q,o,{value:B.n,enumerable:false,writable:true,configurable:true})
return B.n}return B.n},
iE(a,b){if(a<0||a>4294967295)throw A.i(A.aa(a,0,4294967295,"length",null))
return J.iF(new Array(a),b)},
iF(a,b){var s=A.h(a,b.i("n<0>"))
s.$flags=1
return s},
ag(a){if(typeof a=="number"){if(Math.floor(a)==a)return J.bi.prototype
return J.cs.prototype}if(typeof a=="string")return J.aO.prototype
if(a==null)return J.bj.prototype
if(typeof a=="boolean")return J.cq.prototype
if(Array.isArray(a))return J.n.prototype
if(typeof a!="object"){if(typeof a=="function")return J.a4.prototype
if(typeof a=="symbol")return J.bn.prototype
if(typeof a=="bigint")return J.bl.prototype
return a}if(a instanceof A.p)return a
return J.fJ(a)},
hV(a){if(typeof a=="string")return J.aO.prototype
if(a==null)return a
if(Array.isArray(a))return J.n.prototype
if(typeof a!="object"){if(typeof a=="function")return J.a4.prototype
if(typeof a=="symbol")return J.bn.prototype
if(typeof a=="bigint")return J.bl.prototype
return a}if(a instanceof A.p)return a
return J.fJ(a)},
ky(a){if(a==null)return a
if(Array.isArray(a))return J.n.prototype
if(typeof a!="object"){if(typeof a=="function")return J.a4.prototype
if(typeof a=="symbol")return J.bn.prototype
if(typeof a=="bigint")return J.bl.prototype
return a}if(a instanceof A.p)return a
return J.fJ(a)},
df(a,b){if(a==null)return b==null
if(typeof a!="object")return b!=null&&a===b
return J.ag(a).B(a,b)},
T(a){return J.ag(a).gn(a)},
fO(a){return J.ky(a).gbj(a)},
fP(a){return J.hV(a).gA(a)},
io(a){return J.ag(a).gt(a)},
ip(a,b){return J.ag(a).bl(a,b)},
bc(a){return J.ag(a).h(a)},
co:function co(){},
cq:function cq(){},
bj:function bj(){},
bm:function bm(){},
aj:function aj(){},
cF:function cF(){},
bC:function bC(){},
a4:function a4(){},
bl:function bl(){},
bn:function bn(){},
n:function n(a){this.$ti=a},
cp:function cp(){},
dk:function dk(a){this.$ti=a},
c9:function c9(a,b,c){var _=this
_.a=a
_.b=b
_.c=0
_.d=null
_.$ti=c},
bk:function bk(){},
bi:function bi(){},
cs:function cs(){},
aO:function aO(){}},A={fr:function fr(){},
iH(a){return new A.aw("Field '"+a+"' has been assigned during initialization.")},
iJ(a){return new A.aw("Field '"+a+"' has not been initialized.")},
iI(a){return new A.aw("Field '"+a+"' has already been initialized.")},
eZ(a){var s,r=a^48
if(r<=9)return r
s=a|32
if(97<=s&&s<=102)return s-87
return-1},
ac(a,b){a=a+b&536870911
a=a+((a&524287)<<10)&536870911
return a^a>>>6},
dC(a){a=a+((a&67108863)<<3)&536870911
a^=a>>>11
return a+((a&16383)<<15)&536870911},
fH(a,b,c){return a},
hY(a){var s,r
for(s=$.af.length,r=0;r<s;++r)if(a===$.af[r])return!0
return!1},
fZ(){return new A.aX("No element")},
aw:function aw(a){this.a=a},
dy:function dy(){},
cu:function cu(a,b,c){var _=this
_.a=a
_.b=b
_.c=0
_.d=null
_.$ti=c},
L:function L(){},
ao:function ao(a){this.a=a},
i5(a){var s=v.mangledGlobalNames[a]
if(s!=null)return s
return"minified:"+a},
lb(a,b){var s
if(b!=null){s=b.x
if(s!=null)return s}return t.E.b(a)},
r(a){var s
if(typeof a=="string")return a
if(typeof a=="number"){if(a!==0)return""+a}else if(!0===a)return"true"
else if(!1===a)return"false"
else if(a==null)return"null"
s=J.bc(a)
return s},
cG(a){var s,r=$.h4
if(r==null)r=$.h4=Symbol("identityHashCode")
s=a[r]
if(s==null){s=Math.random()*0x3fffffff|0
a[r]=s}return s},
h5(a,b){var s,r=/^\s*[+-]?((0x[a-f0-9]+)|(\d+)|([a-z0-9]+))\s*$/i.exec(a)
if(r==null)return null
if(3>=r.length)return A.f(r,3)
s=r[3]
if(s!=null)return parseInt(a,10)
if(r[2]!=null)return parseInt(a,16)
return null},
cH(a){var s,r,q,p
if(a instanceof A.p)return A.R(A.b9(a),null)
s=J.ag(a)
if(s===B.P||s===B.R||t.cr.b(a)){r=B.p(a)
if(r!=="Object"&&r!=="")return r
q=a.constructor
if(typeof q=="function"){p=q.name
if(typeof p=="string"&&p!=="Object"&&p!=="")return p}}return A.R(A.b9(a),null)},
h6(a){var s,r,q
if(a==null||typeof a=="number"||A.fC(a))return J.bc(a)
if(typeof a=="string")return JSON.stringify(a)
if(a instanceof A.ai)return a.h(0)
if(a instanceof A.a5)return a.aZ(!0)
s=$.im()
for(r=0;r<1;++r){q=s[r].cp(a)
if(q!=null)return q}return"Instance of '"+A.cH(a)+"'"},
iO(){if(!!self.location)return self.location.href
return null},
iQ(a,b,c){var s,r,q,p
if(c<=500&&b===0&&c===a.length)return String.fromCharCode.apply(null,a)
for(s=b,r="";s<c;s=q){q=s+500
p=q<c?q:c
r+=String.fromCharCode.apply(null,a.subarray(s,p))}return r},
ax(a){var s
if(0<=a){if(a<=65535)return String.fromCharCode(a)
if(a<=1114111){s=a-65536
return String.fromCharCode((B.b.R(s,10)|55296)>>>0,s&1023|56320)}}throw A.i(A.aa(a,0,1114111,null,null))},
ak(a,b,c){var s,r,q={}
q.a=0
s=[]
r=[]
q.a=b.length
B.c.S(s,b)
q.b=""
if(c!=null&&c.a!==0)c.T(0,new A.dv(q,r,s))
return J.ip(a,new A.cr(B.a6,0,s,r,0))},
iN(a,b,c){var s,r,q
if(Array.isArray(b))s=c==null||c.a===0
else s=!1
if(s){r=b.length
if(r===0){if(!!a.$0)return a.$0()}else if(r===1){if(!!a.$1)return a.$1(b[0])}else if(r===2){if(!!a.$2)return a.$2(b[0],b[1])}else if(r===3){if(!!a.$3)return a.$3(b[0],b[1],b[2])}else if(r===4){if(!!a.$4)return a.$4(b[0],b[1],b[2],b[3])}else if(r===5)if(!!a.$5)return a.$5(b[0],b[1],b[2],b[3],b[4])
q=a[""+"$"+r]
if(q!=null)return q.apply(a,b)}return A.iM(a,b,c)},
iM(a,b,c){var s,r,q,p,o,n,m,l,k,j,i,h,g,f,e
if(Array.isArray(b))s=b
else s=A.dm(b,t.z)
r=s.length
q=a.$R
if(r<q)return A.ak(a,s,c)
p=a.$D
o=p==null
n=!o?p():null
m=J.ag(a)
l=m.$C
if(typeof l=="string")l=m[l]
if(o){if(c!=null&&c.a!==0)return A.ak(a,s,c)
if(r===q)return l.apply(a,s)
return A.ak(a,s,c)}if(Array.isArray(n)){if(c!=null&&c.a!==0)return A.ak(a,s,c)
k=q+n.length
if(r>k)return A.ak(a,s,null)
if(r<k){j=n.slice(r-q)
if(s===b)s=A.dm(s,t.z)
B.c.S(s,j)}return l.apply(a,s)}else{if(r>q)return A.ak(a,s,c)
if(s===b)s=A.dm(s,t.z)
i=Object.keys(n)
if(c==null)for(o=i.length,h=0;h<i.length;i.length===o||(0,A.S)(i),++h){g=n[A.Q(i[h])]
if(B.r===g)return A.ak(a,s,c)
B.c.m(s,g)}else{for(o=i.length,f=0,h=0;h<i.length;i.length===o||(0,A.S)(i),++h){e=A.Q(i[h])
if(c.bZ(e)){++f
B.c.m(s,c.O(0,e))}else{g=n[e]
if(B.r===g)return A.ak(a,s,c)
B.c.m(s,g)}}if(f!==c.a)return A.ak(a,s,c)}return l.apply(a,s)}},
iP(a){var s=a.$thrownJsError
if(s==null)return null
return A.aH(s)},
h7(a,b){var s
if(a.$thrownJsError==null){s=new Error()
A.F(a,s)
a.$thrownJsError=s
s.stack=b.h(0)}},
kB(a){throw A.i(A.fG(a))},
f(a,b){if(a==null)J.fP(a)
throw A.i(A.hU(a,b))},
hU(a,b){var s,r="index"
if(!A.fE(b))return new A.a3(!0,b,r,null)
s=A.j(J.fP(a))
if(b<0||b>=s)return A.iA(b,s,a,r)
return new A.bx(null,null,!0,b,r,"Value not in range")},
fG(a){return new A.a3(!0,a,null,null)},
i(a){return A.F(a,new Error())},
F(a,b){var s
if(a==null)a=new A.ad()
b.dartException=a
s=A.kM
if("defineProperty" in Object){Object.defineProperty(b,"message",{get:s})
b.name=""}else b.toString=s
return b},
kM(){return J.bc(this.dartException)},
A(a,b){throw A.F(a,b==null?new Error():b)},
ar(a,b,c){var s
if(b==null)b=0
if(c==null)c=0
s=Error()
A.A(A.jT(a,b,c),s)},
jT(a,b,c){var s,r,q,p,o,n,m,l,k
if(typeof b=="string")s=b
else{r="[]=;add;removeWhere;retainWhere;removeRange;setRange;setInt8;setInt16;setInt32;setUint8;setUint16;setUint32;setFloat32;setFloat64".split(";")
q=r.length
p=b
if(p>q){c=p/q|0
p%=q}s=r[p]}o=typeof c=="string"?c:"modify;remove from;add to".split(";")[c]
n=t.j.b(a)?"list":"ByteData"
m=a.$flags|0
l="a "
if((m&4)!==0)k="constant "
else if((m&2)!==0){k="unmodifiable "
l="an "}else k=(m&1)!==0?"fixed-length ":""
return new A.bE("'"+s+"': Cannot "+o+" "+l+k+n)},
S(a){throw A.i(A.cj(a))},
ae(a){var s,r,q,p,o,n
a=A.i2(a.replace(String({}),"$receiver$"))
s=a.match(/\\\$[a-zA-Z]+\\\$/g)
if(s==null)s=A.h([],t.s)
r=s.indexOf("\\$arguments\\$")
q=s.indexOf("\\$argumentsExpr\\$")
p=s.indexOf("\\$expr\\$")
o=s.indexOf("\\$method\\$")
n=s.indexOf("\\$receiver\\$")
return new A.dD(a.replace(new RegExp("\\\\\\$arguments\\\\\\$","g"),"((?:x|[^x])*)").replace(new RegExp("\\\\\\$argumentsExpr\\\\\\$","g"),"((?:x|[^x])*)").replace(new RegExp("\\\\\\$expr\\\\\\$","g"),"((?:x|[^x])*)").replace(new RegExp("\\\\\\$method\\\\\\$","g"),"((?:x|[^x])*)").replace(new RegExp("\\\\\\$receiver\\\\\\$","g"),"((?:x|[^x])*)"),r,q,p,o,n)},
dE(a){return function($expr$){var $argumentsExpr$="$arguments$"
try{$expr$.$method$($argumentsExpr$)}catch(s){return s.message}}(a)},
hb(a){return function($expr$){try{$expr$.$method$}catch(s){return s.message}}(a)},
fs(a,b){var s=b==null,r=s?null:b.method
return new A.ct(a,r,s?null:b.receiver)},
aJ(a){var s
if(a==null)return new A.du(a)
if(a instanceof A.bh){s=a.a
return A.aq(a,s==null?A.b3(s):s)}if(typeof a!=="object")return a
if("dartException" in a)return A.aq(a,a.dartException)
return A.kp(a)},
aq(a,b){if(t.C.b(b))if(b.$thrownJsError==null)b.$thrownJsError=a
return b},
kp(a){var s,r,q,p,o,n,m,l,k,j,i,h,g
if(!("message" in a))return a
s=a.message
if("number" in a&&typeof a.number=="number"){r=a.number
q=r&65535
if((B.b.R(r,16)&8191)===10)switch(q){case 438:return A.aq(a,A.fs(A.r(s)+" (Error "+q+")",null))
case 445:case 5007:A.r(s)
return A.aq(a,new A.bv())}}if(a instanceof TypeError){p=$.i7()
o=$.i8()
n=$.i9()
m=$.ia()
l=$.id()
k=$.ie()
j=$.ic()
$.ib()
i=$.ih()
h=$.ig()
g=p.J(s)
if(g!=null)return A.aq(a,A.fs(A.Q(s),g))
else{g=o.J(s)
if(g!=null){g.method="call"
return A.aq(a,A.fs(A.Q(s),g))}else if(n.J(s)!=null||m.J(s)!=null||l.J(s)!=null||k.J(s)!=null||j.J(s)!=null||m.J(s)!=null||i.J(s)!=null||h.J(s)!=null){A.Q(s)
return A.aq(a,new A.bv())}}return A.aq(a,new A.cO(typeof s=="string"?s:""))}if(a instanceof RangeError){if(typeof s=="string"&&s.indexOf("call stack")!==-1)return new A.bA()
s=function(b){try{return String(b)}catch(f){}return null}(a)
return A.aq(a,new A.a3(!1,null,null,typeof s=="string"?s.replace(/^RangeError:\s*/,""):s))}if(typeof InternalError=="function"&&a instanceof InternalError)if(typeof s=="string"&&s==="too much recursion")return new A.bA()
return a},
aH(a){var s
if(a instanceof A.bh)return a.b
if(a==null)return new A.bT(a)
s=a.$cachedTrace
if(s!=null)return s
s=new A.bT(a)
if(typeof a==="object")a.$cachedTrace=s
return s},
hZ(a){if(a==null)return J.T(a)
if(typeof a=="object")return A.cG(a)
return J.T(a)},
k3(a,b,c,d,e,f){t.Z.a(a)
switch(A.j(b)){case 0:return a.$0()
case 1:return a.$1(c)
case 2:return a.$2(c,d)
case 3:return a.$3(c,d,e)
case 4:return a.$4(c,d,e,f)}throw A.i(new A.es("Unsupported number of arguments for wrapped closure"))},
c7(a,b){var s=a.$identity
if(!!s)return s
s=A.ku(a,b)
a.$identity=s
return s},
ku(a,b){var s
switch(b){case 0:s=a.$0
break
case 1:s=a.$1
break
case 2:s=a.$2
break
case 3:s=a.$3
break
case 4:s=a.$4
break
default:s=null}if(s!=null)return s.bind(a)
return function(c,d,e){return function(f,g,h,i){return e(c,d,f,g,h,i)}}(a,b,A.k3)},
iw(a2){var s,r,q,p,o,n,m,l,k,j,i=a2.co,h=a2.iS,g=a2.iI,f=a2.nDA,e=a2.aI,d=a2.fs,c=a2.cs,b=d[0],a=c[0],a0=i[b],a1=a2.fT
a1.toString
s=h?Object.create(new A.cL().constructor.prototype):Object.create(new A.aL(null,null).constructor.prototype)
s.$initialize=s.constructor
r=h?function static_tear_off(){this.$initialize()}:function tear_off(a3,a4){this.$initialize(a3,a4)}
s.constructor=r
r.prototype=s
s.$_name=b
s.$_target=a0
q=!h
if(q)p=A.fX(b,a0,g,f)
else{s.$static_name=b
p=a0}s.$S=A.is(a1,h,g)
s[a]=p
for(o=p,n=1;n<d.length;++n){m=d[n]
if(typeof m=="string"){l=i[m]
k=m
m=l}else k=""
j=c[n]
if(j!=null){if(q)m=A.fX(k,m,g,f)
s[j]=m}if(n===e)o=m}s.$C=o
s.$R=a2.rC
s.$D=a2.dV
return r},
is(a,b,c){if(typeof a=="number")return a
if(typeof a=="string"){if(b)throw A.i("Cannot compute signature for static tearoff.")
return function(d,e){return function(){return e(this,d)}}(a,A.iq)}throw A.i("Error in functionType of tearoff")},
it(a,b,c,d){var s=A.fV
switch(b?-1:a){case 0:return function(e,f){return function(){return f(this)[e]()}}(c,s)
case 1:return function(e,f){return function(g){return f(this)[e](g)}}(c,s)
case 2:return function(e,f){return function(g,h){return f(this)[e](g,h)}}(c,s)
case 3:return function(e,f){return function(g,h,i){return f(this)[e](g,h,i)}}(c,s)
case 4:return function(e,f){return function(g,h,i,j){return f(this)[e](g,h,i,j)}}(c,s)
case 5:return function(e,f){return function(g,h,i,j,k){return f(this)[e](g,h,i,j,k)}}(c,s)
default:return function(e,f){return function(){return e.apply(f(this),arguments)}}(d,s)}},
fX(a,b,c,d){if(c)return A.iv(a,b,d)
return A.it(b.length,d,a,b)},
iu(a,b,c,d){var s=A.fV,r=A.ir
switch(b?-1:a){case 0:throw A.i(new A.cK("Intercepted function with no arguments."))
case 1:return function(e,f,g){return function(){return f(this)[e](g(this))}}(c,r,s)
case 2:return function(e,f,g){return function(h){return f(this)[e](g(this),h)}}(c,r,s)
case 3:return function(e,f,g){return function(h,i){return f(this)[e](g(this),h,i)}}(c,r,s)
case 4:return function(e,f,g){return function(h,i,j){return f(this)[e](g(this),h,i,j)}}(c,r,s)
case 5:return function(e,f,g){return function(h,i,j,k){return f(this)[e](g(this),h,i,j,k)}}(c,r,s)
case 6:return function(e,f,g){return function(h,i,j,k,l){return f(this)[e](g(this),h,i,j,k,l)}}(c,r,s)
default:return function(e,f,g){return function(){var q=[g(this)]
Array.prototype.push.apply(q,arguments)
return e.apply(f(this),q)}}(d,r,s)}},
iv(a,b,c){var s,r
if($.fT==null)$.fT=A.fS("interceptor")
if($.fU==null)$.fU=A.fS("receiver")
s=b.length
r=A.iu(s,c,a,b)
return r},
fI(a){return A.iw(a)},
iq(a,b){return A.bY(v.typeUniverse,A.b9(a.a),b)},
fV(a){return a.a},
ir(a){return a.b},
fS(a){var s,r,q,p=new A.aL("receiver","interceptor"),o=Object.getOwnPropertyNames(p)
o.$flags=1
s=o
for(o=s.length,r=0;r<o;++r){q=s[r]
if(p[q]===a)return q}throw A.i(A.Y("Field name "+a+" not found.",null))},
hW(a){return v.getIsolateTag(a)},
la(a,b,c){Object.defineProperty(a,b,{value:c,enumerable:false,writable:true,configurable:true})},
kI(a){var s,r,q,p,o,n=A.Q($.hX.$1(a)),m=$.eY[n]
if(m!=null){Object.defineProperty(a,v.dispatchPropertyName,{value:m,enumerable:false,writable:true,configurable:true})
return m.i}s=$.f4[n]
if(s!=null)return s
r=v.interceptorsByTag[n]
if(r==null){q=A.fA($.hR.$2(a,n))
if(q!=null){m=$.eY[q]
if(m!=null){Object.defineProperty(a,v.dispatchPropertyName,{value:m,enumerable:false,writable:true,configurable:true})
return m.i}s=$.f4[q]
if(s!=null)return s
r=v.interceptorsByTag[q]
n=q}}if(r==null)return null
s=r.prototype
p=n[0]
if(p==="!"){m=A.f6(s)
$.eY[n]=m
Object.defineProperty(a,v.dispatchPropertyName,{value:m,enumerable:false,writable:true,configurable:true})
return m.i}if(p==="~"){$.f4[n]=s
return s}if(p==="-"){o=A.f6(s)
Object.defineProperty(Object.getPrototypeOf(a),v.dispatchPropertyName,{value:o,enumerable:false,writable:true,configurable:true})
return o.i}if(p==="+")return A.i0(a,s)
if(p==="*")throw A.i(A.hc(n))
if(v.leafTags[n]===true){o=A.f6(s)
Object.defineProperty(Object.getPrototypeOf(a),v.dispatchPropertyName,{value:o,enumerable:false,writable:true,configurable:true})
return o.i}else return A.i0(a,s)},
i0(a,b){var s=Object.getPrototypeOf(a)
Object.defineProperty(s,v.dispatchPropertyName,{value:J.fL(b,s,null,null),enumerable:false,writable:true,configurable:true})
return b},
f6(a){return J.fL(a,!1,null,!!a.$iO)},
kJ(a,b,c){var s=b.prototype
if(v.leafTags[a]===true)return A.f6(s)
else return J.fL(s,c,null,null)},
kD(){if(!0===$.fK)return
$.fK=!0
A.kE()},
kE(){var s,r,q,p,o,n,m,l
$.eY=Object.create(null)
$.f4=Object.create(null)
A.kC()
s=v.interceptorsByTag
r=Object.getOwnPropertyNames(s)
if(typeof window!="undefined"){window
q=function(){}
for(p=0;p<r.length;++p){o=r[p]
n=$.i1.$1(o)
if(n!=null){m=A.kJ(o,s[o],n)
if(m!=null){Object.defineProperty(n,v.dispatchPropertyName,{value:m,enumerable:false,writable:true,configurable:true})
q.prototype=n}}}}for(p=0;p<r.length;++p){o=r[p]
if(/^[A-Za-z_]/.test(o)){l=s[o]
s["!"+o]=l
s["~"+o]=l
s["-"+o]=l
s["+"+o]=l
s["*"+o]=l}}},
kC(){var s,r,q,p,o,n,m=B.B()
m=A.b8(B.C,A.b8(B.D,A.b8(B.q,A.b8(B.q,A.b8(B.E,A.b8(B.F,A.b8(B.G(B.p),m)))))))
if(typeof dartNativeDispatchHooksTransformer!="undefined"){s=dartNativeDispatchHooksTransformer
if(typeof s=="function")s=[s]
if(Array.isArray(s))for(r=0;r<s.length;++r){q=s[r]
if(typeof q=="function")m=q(m)||m}}p=m.getTag
o=m.getUnknownTag
n=m.prototypeForTag
$.hX=new A.f_(p)
$.hR=new A.f0(o)
$.i1=new A.f1(n)},
b8(a,b){return a(b)||b},
jf(a,b){var s,r
for(s=0;s<a.length;++s){r=a[s]
if(!(s<b.length))return A.f(b,s)
if(!J.df(r,b[s]))return!1}return!0},
kv(a,b){var s=b.length,r=v.rttc[""+s+";"+a]
if(r==null)return null
if(s===0)return r
if(s===r.length)return r.apply(null,b)
return r(b)},
kw(a){if(a.indexOf("$",0)>=0)return a.replace(/\$/g,"$$$$")
return a},
i2(a){if(/[[\]{}()*+?.\\^$|]/.test(a))return a.replace(/[[\]{}()*+?.\\^$|]/g,"\\$&")
return a},
i3(a,b,c){var s=A.kK(a,b,c)
return s},
kK(a,b,c){var s,r,q
if(b===""){if(a==="")return c
s=a.length
for(r=c,q=0;q<s;++q)r=r+a[q]+c
return r.charCodeAt(0)==0?r:r}if(a.indexOf(b,0)<0)return a
if(a.length<500||c.indexOf("$",0)>=0)return a.split(b).join(c)
return a.replace(new RegExp(A.i2(b),"g"),A.kw(c))},
bR:function bR(a){this.a=a},
N:function N(a,b){this.a=a
this.b=b},
bS:function bS(a){this.a=a},
bf:function bf(a,b){this.a=a
this.$ti=b},
be:function be(){},
bg:function bg(a,b,c){this.a=a
this.b=b
this.$ti=c},
cr:function cr(a,b,c,d,e){var _=this
_.a=a
_.c=b
_.d=c
_.e=d
_.f=e},
dv:function dv(a,b,c){this.a=a
this.b=b
this.c=c},
bz:function bz(){},
dD:function dD(a,b,c,d,e,f){var _=this
_.a=a
_.b=b
_.c=c
_.d=d
_.e=e
_.f=f},
bv:function bv(){},
ct:function ct(a,b,c){this.a=a
this.b=b
this.c=c},
cO:function cO(a){this.a=a},
du:function du(a){this.a=a},
bh:function bh(a,b){this.a=a
this.b=b},
bT:function bT(a){this.a=a
this.b=null},
ai:function ai(){},
cf:function cf(){},
cg:function cg(){},
cM:function cM(){},
cL:function cL(){},
aL:function aL(a,b){this.a=a
this.b=b},
cK:function cK(a){this.a=a},
eG:function eG(){},
av:function av(a){var _=this
_.a=0
_.f=_.e=_.d=_.c=_.b=null
_.r=0
_.$ti=a},
dl:function dl(a,b){var _=this
_.a=a
_.b=b
_.d=_.c=null},
f_:function f_(a){this.a=a},
f0:function f0(a){this.a=a},
f1:function f1(a){this.a=a},
a5:function a5(){},
b_:function b_(){},
aC:function aC(){},
l(a){throw A.F(A.iJ(a),new Error())},
kL(a){throw A.F(A.iI(a),new Error())},
i4(a){throw A.F(A.iH(a),new Error())},
j8(a){var s=new A.ep(a)
return s.b=s},
ep:function ep(a){this.a=a
this.b=null},
jU(a){return a},
iL(a){return new Uint8Array(a)},
aR:function aR(){},
aQ:function aQ(){},
bs:function bs(){},
cv:function cv(){},
I:function I(){},
bq:function bq(){},
br:function br(){},
cw:function cw(){},
cx:function cx(){},
cy:function cy(){},
cz:function cz(){},
cA:function cA(){},
cB:function cB(){},
cC:function cC(){},
bt:function bt(){},
bu:function bu(){},
bM:function bM(){},
bN:function bN(){},
bO:function bO(){},
bP:function bP(){},
ft(a,b){var s=b.c
return s==null?b.c=A.bW(a,"at",[b.x]):s},
h9(a){var s=a.w
if(s===6||s===7)return A.h9(a.x)
return s===11||s===12},
iS(a){return a.as},
i_(a,b){var s,r=b.length
for(s=0;s<r;++s)if(!a[s].b(b[s]))return!1
return!0},
aG(a){return A.eM(v.typeUniverse,a,!1)},
aE(a1,a2,a3,a4){var s,r,q,p,o,n,m,l,k,j,i,h,g,f,e,d,c,b,a,a0=a2.w
switch(a0){case 5:case 1:case 2:case 3:case 4:return a2
case 6:s=a2.x
r=A.aE(a1,s,a3,a4)
if(r===s)return a2
return A.hr(a1,r,!0)
case 7:s=a2.x
r=A.aE(a1,s,a3,a4)
if(r===s)return a2
return A.hq(a1,r,!0)
case 8:q=a2.y
p=A.b7(a1,q,a3,a4)
if(p===q)return a2
return A.bW(a1,a2.x,p)
case 9:o=a2.x
n=A.aE(a1,o,a3,a4)
m=a2.y
l=A.b7(a1,m,a3,a4)
if(n===o&&l===m)return a2
return A.fw(a1,n,l)
case 10:k=a2.x
j=a2.y
i=A.b7(a1,j,a3,a4)
if(i===j)return a2
return A.hs(a1,k,i)
case 11:h=a2.x
g=A.aE(a1,h,a3,a4)
f=a2.y
e=A.km(a1,f,a3,a4)
if(g===h&&e===f)return a2
return A.hp(a1,g,e)
case 12:d=a2.y
a4+=d.length
c=A.b7(a1,d,a3,a4)
o=a2.x
n=A.aE(a1,o,a3,a4)
if(c===d&&n===o)return a2
return A.fx(a1,n,c,!0)
case 13:b=a2.x
if(b<a4)return a2
a=a3[b-a4]
if(a==null)return a2
return a
default:throw A.i(A.cb("Attempted to substitute unexpected RTI kind "+a0))}},
b7(a,b,c,d){var s,r,q,p,o=b.length,n=A.eQ(o)
for(s=!1,r=0;r<o;++r){q=b[r]
p=A.aE(a,q,c,d)
if(p!==q)s=!0
n[r]=p}return s?n:b},
kn(a,b,c,d){var s,r,q,p,o,n,m=b.length,l=A.eQ(m)
for(s=!1,r=0;r<m;r+=3){q=b[r]
p=b[r+1]
o=b[r+2]
n=A.aE(a,o,c,d)
if(n!==o)s=!0
l.splice(r,3,q,p,n)}return s?l:b},
km(a,b,c,d){var s,r=b.a,q=A.b7(a,r,c,d),p=b.b,o=A.b7(a,p,c,d),n=b.c,m=A.kn(a,n,c,d)
if(q===r&&o===p&&m===n)return b
s=new A.d0()
s.a=q
s.b=o
s.c=m
return s},
h(a,b){a[v.arrayRti]=b
return a},
hT(a){var s=a.$S
if(s!=null){if(typeof s=="number")return A.kA(s)
return a.$S()}return null},
kF(a,b){var s
if(A.h9(b))if(a instanceof A.ai){s=A.hT(a)
if(s!=null)return s}return A.b9(a)},
b9(a){if(a instanceof A.p)return A.c4(a)
if(Array.isArray(a))return A.b2(a)
return A.fB(J.ag(a))},
b2(a){var s=a[v.arrayRti],r=t.b
if(s==null)return r
if(s.constructor!==r.constructor)return r
return s},
c4(a){var s=a.$ti
return s!=null?s:A.fB(a)},
fB(a){var s=a.constructor,r=s.$ccache
if(r!=null)return r
return A.k0(a,s)},
k0(a,b){var s=a instanceof A.ai?Object.getPrototypeOf(Object.getPrototypeOf(a)).constructor:b,r=A.jo(v.typeUniverse,s.name)
b.$ccache=r
return r},
kA(a){var s,r=v.types,q=r[a]
if(typeof q=="string"){s=A.eM(v.typeUniverse,q,!1)
r[a]=s
return s}return q},
kz(a){return A.aF(A.c4(a))},
fF(a){var s
if(a instanceof A.a5)return A.kx(a.$r,a.ai())
s=a instanceof A.ai?A.hT(a):null
if(s!=null)return s
if(t.bW.b(a))return J.io(a).a
if(Array.isArray(a))return A.b2(a)
return A.b9(a)},
aF(a){var s=a.r
return s==null?a.r=new A.eL(a):s},
kx(a,b){var s,r,q=b,p=q.length
if(p===0)return t.cD
if(0>=p)return A.f(q,0)
s=A.bY(v.typeUniverse,A.fF(q[0]),"@<0>")
for(r=1;r<p;++r){if(!(r<q.length))return A.f(q,r)
s=A.ht(v.typeUniverse,s,A.fF(q[r]))}return A.bY(v.typeUniverse,s,a)},
a2(a){return A.aF(A.eM(v.typeUniverse,a,!1))},
k_(a){var s=this
s.b=A.kk(s)
return s.b(a)},
kk(a){var s,r,q,p,o
if(a===t.K)return A.k9
if(A.aI(a))return A.kd
s=a.w
if(s===6)return A.jY
if(s===1)return A.hK
if(s===7)return A.k4
r=A.kj(a)
if(r!=null)return r
if(s===8){q=a.x
if(a.y.every(A.aI)){a.f="$i"+q
if(q==="k")return A.k7
if(a===t.m)return A.k6
return A.kc}}else if(s===10){p=A.kv(a.x,a.y)
o=p==null?A.hK:p
return o==null?A.b3(o):o}return A.jW},
kj(a){if(a.w===8){if(a===t.S)return A.fE
if(a===t.i||a===t.q)return A.k8
if(a===t.N)return A.kb
if(a===t.y)return A.fC}return null},
jZ(a){var s=this,r=A.jV
if(A.aI(s))r=A.jL
else if(s===t.K)r=A.b3
else if(A.ba(s)){r=A.jX
if(s===t.a3)r=A.jJ
else if(s===t.aD)r=A.fA
else if(s===t.cG)r=A.jH
else if(s===t.ae)r=A.hE
else if(s===t.dd)r=A.jI
else if(s===t.b1)r=A.aD}else if(s===t.S)r=A.j
else if(s===t.N)r=A.Q
else if(s===t.y)r=A.hD
else if(s===t.q)r=A.jK
else if(s===t.i)r=A.c
else if(s===t.m)r=A.t
s.a=r
return s.a(a)},
jW(a){var s=this
if(a==null)return A.ba(s)
return A.kH(v.typeUniverse,A.kF(a,s),s)},
jY(a){if(a==null)return!0
return this.x.b(a)},
kc(a){var s,r=this
if(a==null)return A.ba(r)
s=r.f
if(a instanceof A.p)return!!a[s]
return!!J.ag(a)[s]},
k7(a){var s,r=this
if(a==null)return A.ba(r)
if(typeof a!="object")return!1
if(Array.isArray(a))return!0
s=r.f
if(a instanceof A.p)return!!a[s]
return!!J.ag(a)[s]},
k6(a){var s=this
if(a==null)return!1
if(typeof a=="object"){if(a instanceof A.p)return!!a[s.f]
return!0}if(typeof a=="function")return!0
return!1},
hJ(a){if(typeof a=="object"){if(a instanceof A.p)return t.m.b(a)
return!0}if(typeof a=="function")return!0
return!1},
jV(a){var s=this
if(a==null){if(A.ba(s))return a}else if(s.b(a))return a
throw A.F(A.hF(a,s),new Error())},
jX(a){var s=this
if(a==null||s.b(a))return a
throw A.F(A.hF(a,s),new Error())},
hF(a,b){return new A.bU("TypeError: "+A.hi(a,A.R(b,null)))},
hi(a,b){return A.aN(a)+": type '"+A.R(A.fF(a),null)+"' is not a subtype of type '"+b+"'"},
X(a,b){return new A.bU("TypeError: "+A.hi(a,b))},
k4(a){var s=this
return s.x.b(a)||A.ft(v.typeUniverse,s).b(a)},
k9(a){return a!=null},
b3(a){if(a!=null)return a
throw A.F(A.X(a,"Object"),new Error())},
kd(a){return!0},
jL(a){return a},
hK(a){return!1},
fC(a){return!0===a||!1===a},
hD(a){if(!0===a)return!0
if(!1===a)return!1
throw A.F(A.X(a,"bool"),new Error())},
jH(a){if(!0===a)return!0
if(!1===a)return!1
if(a==null)return a
throw A.F(A.X(a,"bool?"),new Error())},
c(a){if(typeof a=="number")return a
throw A.F(A.X(a,"double"),new Error())},
jI(a){if(typeof a=="number")return a
if(a==null)return a
throw A.F(A.X(a,"double?"),new Error())},
fE(a){return typeof a=="number"&&Math.floor(a)===a},
j(a){if(typeof a=="number"&&Math.floor(a)===a)return a
throw A.F(A.X(a,"int"),new Error())},
jJ(a){if(typeof a=="number"&&Math.floor(a)===a)return a
if(a==null)return a
throw A.F(A.X(a,"int?"),new Error())},
k8(a){return typeof a=="number"},
jK(a){if(typeof a=="number")return a
throw A.F(A.X(a,"num"),new Error())},
hE(a){if(typeof a=="number")return a
if(a==null)return a
throw A.F(A.X(a,"num?"),new Error())},
kb(a){return typeof a=="string"},
Q(a){if(typeof a=="string")return a
throw A.F(A.X(a,"String"),new Error())},
fA(a){if(typeof a=="string")return a
if(a==null)return a
throw A.F(A.X(a,"String?"),new Error())},
t(a){if(A.hJ(a))return a
throw A.F(A.X(a,"JSObject"),new Error())},
aD(a){if(a==null)return a
if(A.hJ(a))return a
throw A.F(A.X(a,"JSObject?"),new Error())},
hN(a,b){var s,r,q
for(s="",r="",q=0;q<a.length;++q,r=", ")s+=r+A.R(a[q],b)
return s},
kf(a,b){var s,r,q,p,o,n,m=a.x,l=a.y
if(""===m)return"("+A.hN(l,b)+")"
s=l.length
r=m.split(",")
q=r.length-s
for(p="(",o="",n=0;n<s;++n,o=", "){p+=o
if(q===0)p+="{"
p+=A.R(l[n],b)
if(q>=0)p+=" "+r[q];++q}return p+"})"},
hG(a3,a4,a5){var s,r,q,p,o,n,m,l,k,j,i,h,g,f,e,d,c,b,a,a0,a1=", ",a2=null
if(a5!=null){s=a5.length
if(a4==null)a4=A.h([],t.s)
else a2=a4.length
r=a4.length
for(q=s;q>0;--q)B.c.m(a4,"T"+(r+q))
for(p=t.X,o="<",n="",q=0;q<s;++q,n=a1){m=a4.length
l=m-1-q
if(!(l>=0))return A.f(a4,l)
o=o+n+a4[l]
k=a5[q]
j=k.w
if(!(j===2||j===3||j===4||j===5||k===p))o+=" extends "+A.R(k,a4)}o+=">"}else o=""
p=a3.x
i=a3.y
h=i.a
g=h.length
f=i.b
e=f.length
d=i.c
c=d.length
b=A.R(p,a4)
for(a="",a0="",q=0;q<g;++q,a0=a1)a+=a0+A.R(h[q],a4)
if(e>0){a+=a0+"["
for(a0="",q=0;q<e;++q,a0=a1)a+=a0+A.R(f[q],a4)
a+="]"}if(c>0){a+=a0+"{"
for(a0="",q=0;q<c;q+=3,a0=a1){a+=a0
if(d[q+1])a+="required "
a+=A.R(d[q+2],a4)+" "+d[q]}a+="}"}if(a2!=null){a4.toString
a4.length=a2}return o+"("+a+") => "+b},
R(a,b){var s,r,q,p,o,n,m,l=a.w
if(l===5)return"erased"
if(l===2)return"dynamic"
if(l===3)return"void"
if(l===1)return"Never"
if(l===4)return"any"
if(l===6){s=a.x
r=A.R(s,b)
q=s.w
return(q===11||q===12?"("+r+")":r)+"?"}if(l===7)return"FutureOr<"+A.R(a.x,b)+">"
if(l===8){p=A.ko(a.x)
o=a.y
return o.length>0?p+("<"+A.hN(o,b)+">"):p}if(l===10)return A.kf(a,b)
if(l===11)return A.hG(a,b,null)
if(l===12)return A.hG(a.x,b,a.y)
if(l===13){n=a.x
m=b.length
n=m-1-n
if(!(n>=0&&n<m))return A.f(b,n)
return b[n]}return"?"},
ko(a){var s=v.mangledGlobalNames[a]
if(s!=null)return s
return"minified:"+a},
jp(a,b){var s=a.tR[b]
while(typeof s=="string")s=a.tR[s]
return s},
jo(a,b){var s,r,q,p,o,n=a.eT,m=n[b]
if(m==null)return A.eM(a,b,!1)
else if(typeof m=="number"){s=m
r=A.bX(a,5,"#")
q=A.eQ(s)
for(p=0;p<s;++p)q[p]=r
o=A.bW(a,b,q)
n[b]=o
return o}else return m},
jn(a,b){return A.hB(a.tR,b)},
jm(a,b){return A.hB(a.eT,b)},
eM(a,b,c){var s,r=a.eC,q=r.get(b)
if(q!=null)return q
s=A.hn(A.hl(a,null,b,!1))
r.set(b,s)
return s},
bY(a,b,c){var s,r,q=b.z
if(q==null)q=b.z=new Map()
s=q.get(c)
if(s!=null)return s
r=A.hn(A.hl(a,b,c,!0))
q.set(c,r)
return r},
ht(a,b,c){var s,r,q,p=b.Q
if(p==null)p=b.Q=new Map()
s=c.as
r=p.get(s)
if(r!=null)return r
q=A.fw(a,b,c.w===9?c.y:[c])
p.set(s,q)
return q},
ap(a,b){b.a=A.jZ
b.b=A.k_
return b},
bX(a,b,c){var s,r,q=a.eC.get(c)
if(q!=null)return q
s=new A.a1(null,null)
s.w=b
s.as=c
r=A.ap(a,s)
a.eC.set(c,r)
return r},
hr(a,b,c){var s,r=b.as+"?",q=a.eC.get(r)
if(q!=null)return q
s=A.jk(a,b,r,c)
a.eC.set(r,s)
return s},
jk(a,b,c,d){var s,r,q
if(d){s=b.w
r=!0
if(!A.aI(b))if(!(b===t.P||b===t.T))if(s!==6)r=s===7&&A.ba(b.x)
if(r)return b
else if(s===1)return t.P}q=new A.a1(null,null)
q.w=6
q.x=b
q.as=c
return A.ap(a,q)},
hq(a,b,c){var s,r=b.as+"/",q=a.eC.get(r)
if(q!=null)return q
s=A.ji(a,b,r,c)
a.eC.set(r,s)
return s},
ji(a,b,c,d){var s,r
if(d){s=b.w
if(A.aI(b)||b===t.K)return b
else if(s===1)return A.bW(a,"at",[b])
else if(b===t.P||b===t.T)return t.bc}r=new A.a1(null,null)
r.w=7
r.x=b
r.as=c
return A.ap(a,r)},
jl(a,b){var s,r,q=""+b+"^",p=a.eC.get(q)
if(p!=null)return p
s=new A.a1(null,null)
s.w=13
s.x=b
s.as=q
r=A.ap(a,s)
a.eC.set(q,r)
return r},
bV(a){var s,r,q,p=a.length
for(s="",r="",q=0;q<p;++q,r=",")s+=r+a[q].as
return s},
jh(a){var s,r,q,p,o,n=a.length
for(s="",r="",q=0;q<n;q+=3,r=","){p=a[q]
o=a[q+1]?"!":":"
s+=r+p+o+a[q+2].as}return s},
bW(a,b,c){var s,r,q,p=b
if(c.length>0)p+="<"+A.bV(c)+">"
s=a.eC.get(p)
if(s!=null)return s
r=new A.a1(null,null)
r.w=8
r.x=b
r.y=c
if(c.length>0)r.c=c[0]
r.as=p
q=A.ap(a,r)
a.eC.set(p,q)
return q},
fw(a,b,c){var s,r,q,p,o,n
if(b.w===9){s=b.x
r=b.y.concat(c)}else{r=c
s=b}q=s.as+(";<"+A.bV(r)+">")
p=a.eC.get(q)
if(p!=null)return p
o=new A.a1(null,null)
o.w=9
o.x=s
o.y=r
o.as=q
n=A.ap(a,o)
a.eC.set(q,n)
return n},
hs(a,b,c){var s,r,q="+"+(b+"("+A.bV(c)+")"),p=a.eC.get(q)
if(p!=null)return p
s=new A.a1(null,null)
s.w=10
s.x=b
s.y=c
s.as=q
r=A.ap(a,s)
a.eC.set(q,r)
return r},
hp(a,b,c){var s,r,q,p,o,n=b.as,m=c.a,l=m.length,k=c.b,j=k.length,i=c.c,h=i.length,g="("+A.bV(m)
if(j>0){s=l>0?",":""
g+=s+"["+A.bV(k)+"]"}if(h>0){s=l>0?",":""
g+=s+"{"+A.jh(i)+"}"}r=n+(g+")")
q=a.eC.get(r)
if(q!=null)return q
p=new A.a1(null,null)
p.w=11
p.x=b
p.y=c
p.as=r
o=A.ap(a,p)
a.eC.set(r,o)
return o},
fx(a,b,c,d){var s,r=b.as+("<"+A.bV(c)+">"),q=a.eC.get(r)
if(q!=null)return q
s=A.jj(a,b,c,r,d)
a.eC.set(r,s)
return s},
jj(a,b,c,d,e){var s,r,q,p,o,n,m,l
if(e){s=c.length
r=A.eQ(s)
for(q=0,p=0;p<s;++p){o=c[p]
if(o.w===1){r[p]=o;++q}}if(q>0){n=A.aE(a,b,r,0)
m=A.b7(a,c,r,0)
return A.fx(a,n,m,c!==m)}}l=new A.a1(null,null)
l.w=12
l.x=b
l.y=c
l.as=d
return A.ap(a,l)},
hl(a,b,c,d){return{u:a,e:b,r:c,s:[],p:0,n:d}},
hn(a){var s,r,q,p,o,n,m,l=a.r,k=a.s
for(s=l.length,r=0;r<s;){q=l.charCodeAt(r)
if(q>=48&&q<=57)r=A.ja(r+1,q,l,k)
else if((((q|32)>>>0)-97&65535)<26||q===95||q===36||q===124)r=A.hm(a,r,l,k,!1)
else if(q===46)r=A.hm(a,r,l,k,!0)
else{++r
switch(q){case 44:break
case 58:k.push(!1)
break
case 33:k.push(!0)
break
case 59:k.push(A.aB(a.u,a.e,k.pop()))
break
case 94:k.push(A.jl(a.u,k.pop()))
break
case 35:k.push(A.bX(a.u,5,"#"))
break
case 64:k.push(A.bX(a.u,2,"@"))
break
case 126:k.push(A.bX(a.u,3,"~"))
break
case 60:k.push(a.p)
a.p=k.length
break
case 62:A.jc(a,k)
break
case 38:A.jb(a,k)
break
case 63:p=a.u
k.push(A.hr(p,A.aB(p,a.e,k.pop()),a.n))
break
case 47:p=a.u
k.push(A.hq(p,A.aB(p,a.e,k.pop()),a.n))
break
case 40:k.push(-3)
k.push(a.p)
a.p=k.length
break
case 41:A.j9(a,k)
break
case 91:k.push(a.p)
a.p=k.length
break
case 93:o=k.splice(a.p)
A.ho(a.u,a.e,o)
a.p=k.pop()
k.push(o)
k.push(-1)
break
case 123:k.push(a.p)
a.p=k.length
break
case 125:o=k.splice(a.p)
A.je(a.u,a.e,o)
a.p=k.pop()
k.push(o)
k.push(-2)
break
case 43:n=l.indexOf("(",r)
k.push(l.substring(r,n))
k.push(-4)
k.push(a.p)
a.p=k.length
r=n+1
break
default:throw"Bad character "+q}}}m=k.pop()
return A.aB(a.u,a.e,m)},
ja(a,b,c,d){var s,r,q=b-48
for(s=c.length;a<s;++a){r=c.charCodeAt(a)
if(!(r>=48&&r<=57))break
q=q*10+(r-48)}d.push(q)
return a},
hm(a,b,c,d,e){var s,r,q,p,o,n,m=b+1
for(s=c.length;m<s;++m){r=c.charCodeAt(m)
if(r===46){if(e)break
e=!0}else{if(!((((r|32)>>>0)-97&65535)<26||r===95||r===36||r===124))q=r>=48&&r<=57
else q=!0
if(!q)break}}p=c.substring(b,m)
if(e){s=a.u
o=a.e
if(o.w===9)o=o.x
n=A.jp(s,o.x)[p]
if(n==null)A.A('No "'+p+'" in "'+A.iS(o)+'"')
d.push(A.bY(s,o,n))}else d.push(p)
return m},
jc(a,b){var s,r=a.u,q=A.hk(a,b),p=b.pop()
if(typeof p=="string")b.push(A.bW(r,p,q))
else{s=A.aB(r,a.e,p)
switch(s.w){case 11:b.push(A.fx(r,s,q,a.n))
break
default:b.push(A.fw(r,s,q))
break}}},
j9(a,b){var s,r,q,p=a.u,o=b.pop(),n=null,m=null
if(typeof o=="number")switch(o){case-1:n=b.pop()
break
case-2:m=b.pop()
break
default:b.push(o)
break}else b.push(o)
s=A.hk(a,b)
o=b.pop()
switch(o){case-3:o=b.pop()
if(n==null)n=p.sEA
if(m==null)m=p.sEA
r=A.aB(p,a.e,o)
q=new A.d0()
q.a=s
q.b=n
q.c=m
b.push(A.hp(p,r,q))
return
case-4:b.push(A.hs(p,b.pop(),s))
return
default:throw A.i(A.cb("Unexpected state under `()`: "+A.r(o)))}},
jb(a,b){var s=b.pop()
if(0===s){b.push(A.bX(a.u,1,"0&"))
return}if(1===s){b.push(A.bX(a.u,4,"1&"))
return}throw A.i(A.cb("Unexpected extended operation "+A.r(s)))},
hk(a,b){var s=b.splice(a.p)
A.ho(a.u,a.e,s)
a.p=b.pop()
return s},
aB(a,b,c){if(typeof c=="string")return A.bW(a,c,a.sEA)
else if(typeof c=="number"){b.toString
return A.jd(a,b,c)}else return c},
ho(a,b,c){var s,r=c.length
for(s=0;s<r;++s)c[s]=A.aB(a,b,c[s])},
je(a,b,c){var s,r=c.length
for(s=2;s<r;s+=3)c[s]=A.aB(a,b,c[s])},
jd(a,b,c){var s,r,q=b.w
if(q===9){if(c===0)return b.x
s=b.y
r=s.length
if(c<=r)return s[c-1]
c-=r
b=b.x
q=b.w}else if(c===0)return b
if(q!==8)throw A.i(A.cb("Indexed base must be an interface type"))
s=b.y
if(c<=s.length)return s[c-1]
throw A.i(A.cb("Bad index "+c+" for "+b.h(0)))},
kH(a,b,c){var s,r=b.d
if(r==null)r=b.d=new Map()
s=r.get(c)
if(s==null){s=A.G(a,b,null,c,null)
r.set(c,s)}return s},
G(a,b,c,d,e){var s,r,q,p,o,n,m,l,k,j,i
if(b===d)return!0
if(A.aI(d))return!0
s=b.w
if(s===4)return!0
if(A.aI(b))return!1
if(b.w===1)return!0
r=s===13
if(r)if(A.G(a,c[b.x],c,d,e))return!0
q=d.w
p=t.P
if(b===p||b===t.T){if(q===7)return A.G(a,b,c,d.x,e)
return d===p||d===t.T||q===6}if(d===t.K){if(s===7)return A.G(a,b.x,c,d,e)
return s!==6}if(s===7){if(!A.G(a,b.x,c,d,e))return!1
return A.G(a,A.ft(a,b),c,d,e)}if(s===6)return A.G(a,p,c,d,e)&&A.G(a,b.x,c,d,e)
if(q===7){if(A.G(a,b,c,d.x,e))return!0
return A.G(a,b,c,A.ft(a,d),e)}if(q===6)return A.G(a,b,c,p,e)||A.G(a,b,c,d.x,e)
if(r)return!1
p=s!==11
if((!p||s===12)&&d===t.Z)return!0
o=s===10
if(o&&d===t.cY)return!0
if(q===12){if(b===t.g)return!0
if(s!==12)return!1
n=b.y
m=d.y
l=n.length
if(l!==m.length)return!1
c=c==null?n:n.concat(c)
e=e==null?m:m.concat(e)
for(k=0;k<l;++k){j=n[k]
i=m[k]
if(!A.G(a,j,c,i,e)||!A.G(a,i,e,j,c))return!1}return A.hI(a,b.x,c,d.x,e)}if(q===11){if(b===t.g)return!0
if(p)return!1
return A.hI(a,b,c,d,e)}if(s===8){if(q!==8)return!1
return A.k5(a,b,c,d,e)}if(o&&q===10)return A.ka(a,b,c,d,e)
return!1},
hI(a3,a4,a5,a6,a7){var s,r,q,p,o,n,m,l,k,j,i,h,g,f,e,d,c,b,a,a0,a1,a2
if(!A.G(a3,a4.x,a5,a6.x,a7))return!1
s=a4.y
r=a6.y
q=s.a
p=r.a
o=q.length
n=p.length
if(o>n)return!1
m=n-o
l=s.b
k=r.b
j=l.length
i=k.length
if(o+j<n+i)return!1
for(h=0;h<o;++h){g=q[h]
if(!A.G(a3,p[h],a7,g,a5))return!1}for(h=0;h<m;++h){g=l[h]
if(!A.G(a3,p[o+h],a7,g,a5))return!1}for(h=0;h<i;++h){g=l[m+h]
if(!A.G(a3,k[h],a7,g,a5))return!1}f=s.c
e=r.c
d=f.length
c=e.length
for(b=0,a=0;a<c;a+=3){a0=e[a]
for(;;){if(b>=d)return!1
a1=f[b]
b+=3
if(a0<a1)return!1
a2=f[b-2]
if(a1<a0){if(a2)return!1
continue}g=e[a+1]
if(a2&&!g)return!1
g=f[b-1]
if(!A.G(a3,e[a+2],a7,g,a5))return!1
break}}while(b<d){if(f[b+1])return!1
b+=3}return!0},
k5(a,b,c,d,e){var s,r,q,p,o,n=b.x,m=d.x
while(n!==m){s=a.tR[n]
if(s==null)return!1
if(typeof s=="string"){n=s
continue}r=s[m]
if(r==null)return!1
q=r.length
p=q>0?new Array(q):v.typeUniverse.sEA
for(o=0;o<q;++o)p[o]=A.bY(a,b,r[o])
return A.hC(a,p,null,c,d.y,e)}return A.hC(a,b.y,null,c,d.y,e)},
hC(a,b,c,d,e,f){var s,r=b.length
for(s=0;s<r;++s)if(!A.G(a,b[s],d,e[s],f))return!1
return!0},
ka(a,b,c,d,e){var s,r=b.y,q=d.y,p=r.length
if(p!==q.length)return!1
if(b.x!==d.x)return!1
for(s=0;s<p;++s)if(!A.G(a,r[s],c,q[s],e))return!1
return!0},
ba(a){var s=a.w,r=!0
if(!(a===t.P||a===t.T))if(!A.aI(a))if(s!==6)r=s===7&&A.ba(a.x)
return r},
aI(a){var s=a.w
return s===2||s===3||s===4||s===5||a===t.X},
hB(a,b){var s,r,q=Object.keys(b),p=q.length
for(s=0;s<p;++s){r=q[s]
a[r]=b[r]}},
eQ(a){return a>0?new Array(a):v.typeUniverse.sEA},
a1:function a1(a,b){var _=this
_.a=a
_.b=b
_.r=_.f=_.d=_.c=null
_.w=0
_.as=_.Q=_.z=_.y=_.x=null},
d0:function d0(){this.c=this.b=this.a=null},
eL:function eL(a){this.a=a},
d_:function d_(){},
bU:function bU(a){this.a=a},
j4(){var s,r,q
if(self.scheduleImmediate!=null)return A.kr()
if(self.MutationObserver!=null&&self.document!=null){s={}
r=self.document.createElement("div")
q=self.document.createElement("span")
s.a=null
new self.MutationObserver(A.c7(new A.em(s),1)).observe(r,{childList:true})
return new A.el(s,r,q)}else if(self.setImmediate!=null)return A.ks()
return A.kt()},
j5(a){self.scheduleImmediate(A.c7(new A.en(t.M.a(a)),0))},
j6(a){self.setImmediate(A.c7(new A.eo(t.M.a(a)),0))},
j7(a){t.M.a(a)
A.jg(0,a)},
jg(a,b){var s=new A.eJ()
s.bB(a,b)
return s},
d9(a){return new A.cV(new A.E($.y,a.i("E<0>")),a.i("cV<0>"))},
d7(a,b){a.$2(0,null)
b.b=!0
return b.a},
b4(a,b){A.jM(a,b)},
d6(a,b){b.ar(a)},
d5(a,b){b.au(A.aJ(a),A.aH(a))},
jM(a,b){var s,r,q=new A.eR(b),p=new A.eS(b)
if(a instanceof A.E)a.aY(q,p,t.z)
else{s=t.z
if(a instanceof A.E)a.bs(q,p,s)
else{r=new A.E($.y,t.c)
r.a=8
r.c=a
r.aY(q,p,s)}}},
db(a){var s=function(b,c){return function(d,e){while(true){try{b(d,e)
break}catch(r){e=r
d=c}}}}(a,1)
return $.y.bq(new A.eX(s),t.H,t.S,t.z)},
fo(a){var s
if(t.C.b(a)){s=a.gX()
if(s!=null)return s}return B.l},
k1(a,b){if($.y===B.i)return null
return null},
k2(a,b){if($.y!==B.i)A.k1(a,b)
if(b==null)if(t.C.b(a)){b=a.gX()
if(b==null){A.h7(a,B.l)
b=B.l}}else b=B.l
else if(t.C.b(a))A.h7(a,b)
return new A.U(a,b)},
fv(a,b,c){var s,r,q,p,o={},n=o.a=a
for(s=t.c;r=n.a,(r&4)!==0;n=a){a=s.a(n.c)
o.a=a}if(n===b){s=A.iT()
b.ac(new A.U(new A.a3(!0,n,null,"Cannot complete a future with itself"),s))
return}q=b.a&1
s=n.a=r|q
if((s&24)===0){p=t.d.a(b.c)
b.a=b.a&1|4
b.c=n
n.aU(p)
return}if(!c)if(b.c==null)n=(s&16)===0||q!==0
else n=!1
else n=!0
if(n){p=b.Y()
b.a2(o.a)
A.aA(b,p)
return}b.a^=2
A.da(null,null,b.b,t.M.a(new A.ew(o,b)))},
aA(a,b){var s,r,q,p,o,n,m,l,k,j,i,h,g,f,e,d={},c=d.a=a
for(s=t.n,r=t.d;;){q={}
p=c.a
o=(p&16)===0
n=!o
if(b==null){if(n&&(p&1)===0){m=s.a(c.c)
A.eV(m.a,m.b)}return}q.a=b
l=b.a
for(c=b;l!=null;c=l,l=k){c.a=null
A.aA(d.a,c)
q.a=l
k=l.a}p=d.a
j=p.c
q.b=n
q.c=j
if(o){i=c.c
i=(i&1)!==0||(i&15)===8}else i=!0
if(i){h=c.b.b
if(n){p=p.b===h
p=!(p||p)}else p=!1
if(p){s.a(j)
A.eV(j.a,j.b)
return}g=$.y
if(g!==h)$.y=h
else g=null
c=c.c
if((c&15)===8)new A.eA(q,d,n).$0()
else if(o){if((c&1)!==0)new A.ez(q,j).$0()}else if((c&2)!==0)new A.ey(d,q).$0()
if(g!=null)$.y=g
c=q.c
if(c instanceof A.E){p=q.a.$ti
p=p.i("at<2>").b(c)||!p.y[1].b(c)}else p=!1
if(p){f=q.a.b
if((c.a&24)!==0){e=r.a(f.c)
f.c=null
b=f.a4(e)
f.a=c.a&30|f.a&1
f.c=c.c
d.a=c
continue}else A.fv(c,f,!0)
return}}f=q.a.b
e=r.a(f.c)
f.c=null
b=f.a4(e)
c=q.b
p=q.c
if(!c){f.$ti.c.a(p)
f.a=8
f.c=p}else{s.a(p)
f.a=f.a&1|16
f.c=p}d.a=f
c=f}},
kg(a,b){var s
if(t.Q.b(a))return b.bq(a,t.z,t.K,t.l)
s=t.v
if(s.b(a))return s.a(a)
throw A.i(A.fQ(a,"onError",u.c))},
ke(){var s,r
for(s=$.b6;s!=null;s=$.b6){$.c6=null
r=s.b
$.b6=r
if(r==null)$.c5=null
s.a.$0()}},
kl(){$.fD=!0
try{A.ke()}finally{$.c6=null
$.fD=!1
if($.b6!=null)$.fN().$1(A.hS())}},
hP(a){var s=new A.cW(a),r=$.c5
if(r==null){$.b6=$.c5=s
if(!$.fD)$.fN().$1(A.hS())}else $.c5=r.b=s},
ki(a){var s,r,q,p=$.b6
if(p==null){A.hP(a)
$.c6=$.c5
return}s=new A.cW(a)
r=$.c6
if(r==null){s.b=p
$.b6=$.c6=s}else{q=r.b
s.b=q
$.c6=r.b=s
if(q==null)$.c5=s}},
kT(a,b){A.fH(a,"stream",t.K)
return new A.d3(b.i("d3<0>"))},
eV(a,b){A.ki(new A.eW(a,b))},
hL(a,b,c,d,e){var s,r=$.y
if(r===c)return d.$0()
$.y=c
s=r
try{r=d.$0()
return r}finally{$.y=s}},
hM(a,b,c,d,e,f,g){var s,r=$.y
if(r===c)return d.$1(e)
$.y=c
s=r
try{r=d.$1(e)
return r}finally{$.y=s}},
kh(a,b,c,d,e,f,g,h,i){var s,r=$.y
if(r===c)return d.$2(e,f)
$.y=c
s=r
try{r=d.$2(e,f)
return r}finally{$.y=s}},
da(a,b,c,d){t.M.a(d)
if(B.i!==c){d=c.bW(d)
d=d}A.hP(d)},
em:function em(a){this.a=a},
el:function el(a,b,c){this.a=a
this.b=b
this.c=c},
en:function en(a){this.a=a},
eo:function eo(a){this.a=a},
eJ:function eJ(){},
eK:function eK(a,b){this.a=a
this.b=b},
cV:function cV(a,b){this.a=a
this.b=!1
this.$ti=b},
eR:function eR(a){this.a=a},
eS:function eS(a){this.a=a},
eX:function eX(a){this.a=a},
U:function U(a,b){this.a=a
this.b=b},
cX:function cX(){},
bJ:function bJ(a,b){this.a=a
this.$ti=b},
az:function az(a,b,c,d,e){var _=this
_.a=null
_.b=a
_.c=b
_.d=c
_.e=d
_.$ti=e},
E:function E(a,b){var _=this
_.a=0
_.b=a
_.c=null
_.$ti=b},
et:function et(a,b){this.a=a
this.b=b},
ex:function ex(a,b){this.a=a
this.b=b},
ew:function ew(a,b){this.a=a
this.b=b},
ev:function ev(a,b){this.a=a
this.b=b},
eu:function eu(a,b){this.a=a
this.b=b},
eA:function eA(a,b,c){this.a=a
this.b=b
this.c=c},
eB:function eB(a,b){this.a=a
this.b=b},
eC:function eC(a){this.a=a},
ez:function ez(a,b){this.a=a
this.b=b},
ey:function ey(a,b){this.a=a
this.b=b},
cW:function cW(a){this.a=a
this.b=null},
bB:function bB(){},
dA:function dA(a,b){this.a=a
this.b=b},
dB:function dB(a,b){this.a=a
this.b=b},
d3:function d3(a){this.$ti=a},
c1:function c1(){},
d1:function d1(){},
eH:function eH(a,b){this.a=a
this.b=b},
eI:function eI(a,b,c){this.a=a
this.b=b
this.c=c},
eW:function eW(a,b){this.a=a
this.b=b},
B(a,b){return new A.av(a.i("@<0>").F(b).i("av<1,2>"))},
dn(a){var s,r
if(A.hY(a))return"{...}"
s=new A.J("")
try{r={}
B.c.m($.af,a)
s.a+="{"
r.a=!0
a.T(0,new A.dp(r,s))
s.a+="}"}finally{if(0>=$.af.length)return A.f($.af,-1)
$.af.pop()}r=s.a
return r.charCodeAt(0)==0?r:r},
x:function x(){},
bp:function bp(){},
dp:function dp(a,b){this.a=a
this.b=b},
bZ:function bZ(){},
aP:function aP(){},
bD:function bD(){},
b0:function b0(){},
jF(a,b,c){var s,r,q,p,o=c-b
if(o<=4096)s=$.il()
else s=new Uint8Array(o)
for(r=0;r<o;++r){q=b+r
if(!(q<a.length))return A.f(a,q)
p=a[q]
if((p&255)!==p)p=255
s[r]=p}return s},
jE(a,b,c,d){var s=a?$.ik():$.ij()
if(s==null)return null
if(0===c&&d===b.length)return A.hA(s,b)
return A.hA(s,b.subarray(c,d))},
hA(a,b){var s,r
try{s=a.decode(b)
return s}catch(r){}return null},
fR(a,b,c,d,e,f){if(B.b.G(f,4)!==0)throw A.i(A.M("Invalid base64 padding, padded length must be multiple of four, is "+f,a,c))
if(d+e!==f)throw A.i(A.M("Invalid base64 padding, '=' not at the end",a,b))
if(e>2)throw A.i(A.M("Invalid base64 padding, more than two '=' characters",a,b))},
jG(a){switch(a){case 65:return"Missing extension byte"
case 67:return"Unexpected extension byte"
case 69:return"Invalid UTF-8 byte"
case 71:return"Overlong encoding"
case 73:return"Out of unicode range"
case 75:return"Encoded surrogate"
case 77:return"Unfinished UTF-8 octet sequence"
default:return""}},
eP:function eP(){},
eO:function eO(){},
cc:function cc(){},
dg:function dg(){},
ch:function ch(){},
ck:function ck(){},
dH:function dH(a){this.a=a},
eN:function eN(a){this.a=a
this.b=16
this.c=0},
kG(a){var s=A.h5(a,null)
if(s!=null)return s
throw A.i(A.M(a,null,null))},
ix(a,b){a=A.F(a,new Error())
if(a==null)a=A.b3(a)
a.stack=b.h(0)
throw a},
h0(a,b,c,d){var s,r=J.iE(a,d)
if(a!==0&&b!=null)for(s=0;s<a;++s)r[s]=b
return r},
iK(a,b,c){var s,r,q=A.h([],c.i("n<0>"))
for(s=a.length,r=0;r<a.length;a.length===s||(0,A.S)(a),++r)B.c.m(q,c.a(a[r]))
q.$flags=1
return q},
dm(a,b){var s,r
if(Array.isArray(a))return A.h(a.slice(0),b.i("n<0>"))
s=A.h([],b.i("n<0>"))
for(r=J.fO(a);r.a_();)B.c.m(s,r.gZ())
return s},
ha(a,b,c){var s,r
A.iR(b,"start")
if(c!=null){s=c-b
if(s<0)throw A.i(A.aa(c,b,null,"end",null))
if(s===0)return""}r=A.iX(a,b,c)
return r},
iX(a,b,c){var s=a.length
if(b>=s)return""
return A.iQ(a,b,c==null||c>s?s:c)},
iV(a,b,c){var s=J.fO(b)
if(!s.a_())return a
if(c.length===0){do a+=A.r(s.gZ())
while(s.a_())}else{a+=A.r(s.gZ())
while(s.a_())a=a+c+A.r(s.gZ())}return a},
h1(a,b){return new A.cD(a,b.gcd(),b.gcj(),b.gce())},
j3(){var s,r,q=A.iO()
if(q==null)throw A.i(A.bF("'Uri.base' is not supported"))
s=$.hf
if(s!=null&&q===$.he)return s
r=A.cQ(q)
$.hf=r
$.he=q
return r},
iT(){return A.aH(new Error())},
aN(a){if(typeof a=="number"||A.fC(a)||a==null)return J.bc(a)
if(typeof a=="string")return JSON.stringify(a)
return A.h6(a)},
iy(a,b){A.fH(a,"error",t.K)
A.fH(b,"stackTrace",t.l)
A.ix(a,b)},
cb(a){return new A.ca(a)},
Y(a,b){return new A.a3(!1,null,b,a)},
fQ(a,b,c){return new A.a3(!0,a,b,c)},
aa(a,b,c,d,e){return new A.bx(b,c,!0,a,d,"Invalid value")},
cI(a,b,c){if(0>a||a>c)throw A.i(A.aa(a,0,c,"start",null))
if(b!=null){if(a>b||b>c)throw A.i(A.aa(b,a,c,"end",null))
return b}return c},
iR(a,b){if(a<0)throw A.i(A.aa(a,0,null,b,null))
return a},
iA(a,b,c,d){return new A.cn(b,!0,a,d,"Index out of range")},
bF(a){return new A.bE(a)},
hc(a){return new A.cN(a)},
C(a){return new A.aX(a)},
cj(a){return new A.ci(a)},
M(a,b,c){return new A.a8(a,b,c)},
h_(a,b,c){var s,r
if(A.hY(a))return b+"..."+c
s=new A.J(b)
B.c.m($.af,a)
try{r=s
r.a=A.iV(r.a,a,", ")}finally{if(0>=$.af.length)return A.f($.af,-1)
$.af.pop()}s.a+=c
r=s.a
return r.charCodeAt(0)==0?r:r},
aS(a,b,c,d){var s
if(B.h===c){s=J.T(a)
b=J.T(b)
return A.dC(A.ac(A.ac($.dd(),s),b))}if(B.h===d){s=J.T(a)
b=J.T(b)
c=J.T(c)
return A.dC(A.ac(A.ac(A.ac($.dd(),s),b),c))}s=J.T(a)
b=J.T(b)
c=J.T(c)
d=J.T(d)
d=A.dC(A.ac(A.ac(A.ac(A.ac($.dd(),s),b),c),d))
return d},
h2(a){var s,r,q=$.dd()
for(s=a.length,r=0;r<a.length;a.length===s||(0,A.S)(a),++r)q=A.ac(q,J.T(a[r]))
return A.dC(q)},
cQ(a5){var s,r,q,p,o,n,m,l,k,j,i,h,g,f,e,d,c,b,a,a0,a1,a2,a3=null,a4=a5.length
if(a4>=5){if(4>=a4)return A.f(a5,4)
s=((a5.charCodeAt(4)^58)*3|a5.charCodeAt(0)^100|a5.charCodeAt(1)^97|a5.charCodeAt(2)^116|a5.charCodeAt(3)^97)>>>0
if(s===0)return A.hd(a4<a4?B.a.l(a5,0,a4):a5,5,a3).gbt()
else if(s===32)return A.hd(B.a.l(a5,5,a4),0,a3).gbt()}r=A.h0(8,0,!1,t.S)
B.c.p(r,0,0)
B.c.p(r,1,-1)
B.c.p(r,2,-1)
B.c.p(r,7,-1)
B.c.p(r,3,0)
B.c.p(r,4,0)
B.c.p(r,5,a4)
B.c.p(r,6,a4)
if(A.hO(a5,0,a4,0,r)>=14)B.c.p(r,7,a4)
q=r[1]
if(q>=0)if(A.hO(a5,0,q,20,r)===20)r[7]=q
p=r[2]+1
o=r[3]
n=r[4]
m=r[5]
l=r[6]
if(l<m)m=l
if(n<p)n=m
else if(n<=q)n=q+1
if(o<p)o=n
k=r[7]<0
j=a3
if(k){k=!1
if(!(p>q+3)){i=o>0
if(!(i&&o+1===n)){if(!B.a.u(a5,"\\",n))if(p>0)h=B.a.u(a5,"\\",p-1)||B.a.u(a5,"\\",p-2)
else h=!1
else h=!0
if(!h){if(!(m<a4&&m===n+2&&B.a.u(a5,"..",n)))h=m>n+2&&B.a.u(a5,"/..",m-3)
else h=!0
if(!h)if(q===4){if(B.a.u(a5,"file",0)){if(p<=0){if(!B.a.u(a5,"/",n)){g="file:///"
s=3}else{g="file://"
s=2}a5=g+B.a.l(a5,n,a4)
m+=s
l+=s
a4=a5.length
p=7
o=7
n=7}else if(n===m){++l
f=m+1
a5=B.a.U(a5,n,m,"/");++a4
m=f}j="file"}else if(B.a.u(a5,"http",0)){if(i&&o+3===n&&B.a.u(a5,"80",o+1)){l-=3
e=n-3
m-=3
a5=B.a.U(a5,o,n,"")
a4-=3
n=e}j="http"}}else if(q===5&&B.a.u(a5,"https",0)){if(i&&o+4===n&&B.a.u(a5,"443",o+1)){l-=4
e=n-4
m-=4
a5=B.a.U(a5,o,n,"")
a4-=3
n=e}j="https"}k=!h}}}}if(k)return new A.d2(a4<a5.length?B.a.l(a5,0,a4):a5,q,p,o,n,m,l,j)
if(j==null)if(q>0)j=A.jy(a5,0,q)
else{if(q===0)A.b1(a5,0,"Invalid empty scheme")
j=""}d=a3
if(p>0){c=q+3
b=c<p?A.jz(a5,c,p-1):""
a=A.ju(a5,p,o,!1)
i=o+1
if(i<n){a0=A.h5(B.a.l(a5,i,n),a3)
d=A.jw(a0==null?A.A(A.M("Invalid port",a5,i)):a0,j)}}else{a=a3
b=""}a1=A.jv(a5,n,m,a3,j,a!=null)
a2=m<l?A.jx(a5,m+1,l,a3):a3
return A.jq(j,b,a,d,a1,a2,l<a4?A.jt(a5,l+1,a4):a3)},
cP(a,b,c){throw A.i(A.M("Illegal IPv4 address, "+a,b,c))},
j0(a,b,c,d,e){var s,r,q,p,o,n,m,l,k,j="invalid character"
for(s=a.length,r=b,q=r,p=0,o=0;;){if(q>=c)n=0
else{if(!(q>=0&&q<s))return A.f(a,q)
n=a.charCodeAt(q)}m=n^48
if(m<=9){if(o!==0||q===r){o=o*10+m
if(o<=255){++q
continue}A.cP("each part must be in the range 0..255",a,r)}A.cP("parts must not have leading zeros",a,r)}if(q===r){if(q===c)break
A.cP(j,a,q)}l=p+1
k=e+p
d.$flags&2&&A.ar(d)
if(!(k<16))return A.f(d,k)
d[k]=o
if(n===46){if(l<4){++q
p=l
r=q
o=0
continue}break}if(q===c){if(l===4)return
break}A.cP(j,a,q)
p=l}A.cP("IPv4 address should contain exactly 4 parts",a,q)},
j1(a,b,c){var s
if(b===c)throw A.i(A.M("Empty IP address",a,b))
if(!(b>=0&&b<a.length))return A.f(a,b)
if(a.charCodeAt(b)===118){s=A.j2(a,b,c)
if(s!=null)throw A.i(s)
return!1}A.hg(a,b,c)
return!0},
j2(a,b,c){var s,r,q,p,o,n="Missing hex-digit in IPvFuture address",m=u.f;++b
for(s=a.length,r=b;;r=q){if(r<c){q=r+1
if(!(r>=0&&r<s))return A.f(a,r)
p=a.charCodeAt(r)
if((p^48)<=9)continue
o=p|32
if(o>=97&&o<=102)continue
if(p===46){if(q-1===b)return new A.a8(n,a,q)
r=q
break}return new A.a8("Unexpected character",a,q-1)}if(r-1===b)return new A.a8(n,a,r)
return new A.a8("Missing '.' in IPvFuture address",a,r)}if(r===c)return new A.a8("Missing address in IPvFuture address, host, cursor",null,null)
for(;;){if(!(r>=0&&r<s))return A.f(a,r)
p=a.charCodeAt(r)
if(!(p<128))return A.f(m,p)
if((m.charCodeAt(p)&16)!==0){++r
if(r<c)continue
return null}return new A.a8("Invalid IPvFuture address character",a,r)}},
hg(a3,a4,a5){var s,r,q,p,o,n,m,l,k,j,i,h,g,f,e,d,c,b,a,a0,a1="an address must contain at most 8 parts",a2=new A.dG(a3)
if(a5-a4<2)a2.$2("address is too short",null)
s=new Uint8Array(16)
r=a3.length
if(!(a4>=0&&a4<r))return A.f(a3,a4)
q=-1
p=0
if(a3.charCodeAt(a4)===58){o=a4+1
if(!(o<r))return A.f(a3,o)
if(a3.charCodeAt(o)===58){n=a4+2
m=n
q=0
p=1}else{a2.$2("invalid start colon",a4)
n=a4
m=n}}else{n=a4
m=n}for(l=0,k=!0;;){if(n>=a5)j=0
else{if(!(n<r))return A.f(a3,n)
j=a3.charCodeAt(n)}A:{i=j^48
h=!1
if(i<=9)g=i
else{f=j|32
if(f>=97&&f<=102)g=f-87
else break A
k=h}if(n<m+4){l=l*16+g;++n
continue}a2.$2("an IPv6 part can contain a maximum of 4 hex digits",m)}if(n>m){if(j===46){if(k){if(p<=6){A.j0(a3,m,a5,s,p*2)
p+=2
n=a5
break}a2.$2(a1,m)}break}o=p*2
e=B.b.R(l,8)
if(!(o<16))return A.f(s,o)
s[o]=e;++o
if(!(o<16))return A.f(s,o)
s[o]=l&255;++p
if(j===58){if(p<8){++n
m=n
l=0
k=!0
continue}a2.$2(a1,n)}break}if(j===58){if(q<0){d=p+1;++n
q=p
p=d
m=n
continue}a2.$2("only one wildcard `::` is allowed",n)}if(q!==p-1)a2.$2("missing part",n)
break}if(n<a5)a2.$2("invalid character",n)
if(p<8){if(q<0)a2.$2("an address without a wildcard must contain exactly 8 parts",a5)
c=q+1
b=p-c
if(b>0){a=c*2
a0=16-b*2
B.y.bx(s,a0,16,s,a)
B.y.c5(s,a,a0,0)}}return s},
jq(a,b,c,d,e,f,g){return new A.c_(a,b,c,d,e,f,g)},
hu(a){if(a==="http")return 80
if(a==="https")return 443
return 0},
b1(a,b,c){throw A.i(A.M(c,a,b))},
jw(a,b){var s=A.hu(b)
if(a===s)return null
return a},
ju(a,b,c,d){var s,r,q,p,o,n,m,l,k
if(b===c)return""
s=a.length
if(!(b>=0&&b<s))return A.f(a,b)
if(a.charCodeAt(b)===91){r=c-1
if(!(r>=0&&r<s))return A.f(a,r)
if(a.charCodeAt(r)!==93)A.b1(a,b,"Missing end `]` to match `[` in host")
q=b+1
if(!(q<s))return A.f(a,q)
p=""
if(a.charCodeAt(q)!==118){o=A.js(a,q,r)
if(o<r){n=o+1
p=A.hz(a,B.a.u(a,"25",n)?o+3:n,r,"%25")}}else o=r
m=A.j1(a,q,o)
l=B.a.l(a,q,o)
return"["+(m?l.toLowerCase():l)+p+"]"}for(k=b;k<c;++k){if(!(k<s))return A.f(a,k)
if(a.charCodeAt(k)===58){o=B.a.a6(a,"%",b)
o=o>=b&&o<c?o:c
if(o<c){n=o+1
p=A.hz(a,B.a.u(a,"25",n)?o+3:n,c,"%25")}else p=""
A.hg(a,b,o)
return"["+B.a.l(a,b,o)+p+"]"}}return A.jB(a,b,c)},
js(a,b,c){var s=B.a.a6(a,"%",b)
return s>=b&&s<c?s:c},
hz(a,b,c,d){var s,r,q,p,o,n,m,l,k,j,i,h=d!==""?new A.J(d):null
for(s=a.length,r=b,q=r,p=!0;r<c;){if(!(r>=0&&r<s))return A.f(a,r)
o=a.charCodeAt(r)
if(o===37){n=A.fz(a,r,!0)
m=n==null
if(m&&p){r+=3
continue}if(h==null)h=new A.J("")
l=h.a+=B.a.l(a,q,r)
if(m)n=B.a.l(a,r,r+3)
else if(n==="%")A.b1(a,r,"ZoneID should not contain % anymore")
h.a=l+n
r+=3
q=r
p=!0}else if(o<127&&(u.f.charCodeAt(o)&1)!==0){if(p&&65<=o&&90>=o){if(h==null)h=new A.J("")
if(q<r){h.a+=B.a.l(a,q,r)
q=r}p=!1}++r}else{k=1
if((o&64512)===55296&&r+1<c){m=r+1
if(!(m<s))return A.f(a,m)
j=a.charCodeAt(m)
if((j&64512)===56320){o=65536+((o&1023)<<10)+(j&1023)
k=2}}i=B.a.l(a,q,r)
if(h==null){h=new A.J("")
m=h}else m=h
m.a+=i
l=A.fy(o)
m.a+=l
r+=k
q=r}}if(h==null)return B.a.l(a,b,c)
if(q<c){i=B.a.l(a,q,c)
h.a+=i}s=h.a
return s.charCodeAt(0)==0?s:s},
jB(a,b,c){var s,r,q,p,o,n,m,l,k,j,i,h,g=u.f
for(s=a.length,r=b,q=r,p=null,o=!0;r<c;){if(!(r>=0&&r<s))return A.f(a,r)
n=a.charCodeAt(r)
if(n===37){m=A.fz(a,r,!0)
l=m==null
if(l&&o){r+=3
continue}if(p==null)p=new A.J("")
k=B.a.l(a,q,r)
if(!o)k=k.toLowerCase()
j=p.a+=k
i=3
if(l)m=B.a.l(a,r,r+3)
else if(m==="%"){m="%25"
i=1}p.a=j+m
r+=i
q=r
o=!0}else if(n<127&&(g.charCodeAt(n)&32)!==0){if(o&&65<=n&&90>=n){if(p==null)p=new A.J("")
if(q<r){p.a+=B.a.l(a,q,r)
q=r}o=!1}++r}else if(n<=93&&(g.charCodeAt(n)&1024)!==0)A.b1(a,r,"Invalid character")
else{i=1
if((n&64512)===55296&&r+1<c){l=r+1
if(!(l<s))return A.f(a,l)
h=a.charCodeAt(l)
if((h&64512)===56320){n=65536+((n&1023)<<10)+(h&1023)
i=2}}k=B.a.l(a,q,r)
if(!o)k=k.toLowerCase()
if(p==null){p=new A.J("")
l=p}else l=p
l.a+=k
j=A.fy(n)
l.a+=j
r+=i
q=r}}if(p==null)return B.a.l(a,b,c)
if(q<c){k=B.a.l(a,q,c)
if(!o)k=k.toLowerCase()
p.a+=k}s=p.a
return s.charCodeAt(0)==0?s:s},
jy(a,b,c){var s,r,q,p
if(b===c)return""
s=a.length
if(!(b<s))return A.f(a,b)
if(!A.hw(a.charCodeAt(b)))A.b1(a,b,"Scheme not starting with alphabetic character")
for(r=b,q=!1;r<c;++r){if(!(r<s))return A.f(a,r)
p=a.charCodeAt(r)
if(!(p<128&&(u.f.charCodeAt(p)&8)!==0))A.b1(a,r,"Illegal scheme character")
if(65<=p&&p<=90)q=!0}a=B.a.l(a,b,c)
return A.jr(q?a.toLowerCase():a)},
jr(a){if(a==="http")return"http"
if(a==="file")return"file"
if(a==="https")return"https"
if(a==="package")return"package"
return a},
jz(a,b,c){return A.c0(a,b,c,16,!1,!1)},
jv(a,b,c,d,e,f){var s=e==="file",r=s||f,q=A.c0(a,b,c,128,!0,!0)
if(q.length===0){if(s)return"/"}else if(r&&!B.a.E(q,"/"))q="/"+q
return A.jA(q,e,f)},
jA(a,b,c){var s=b.length===0
if(s&&!c&&!B.a.E(a,"/")&&!B.a.E(a,"\\"))return A.jC(a,!s||c)
return A.jD(a)},
jx(a,b,c,d){return A.c0(a,b,c,256,!0,!1)},
jt(a,b,c){return A.c0(a,b,c,256,!0,!1)},
fz(a,b,c){var s,r,q,p,o,n,m=u.f,l=b+2,k=a.length
if(l>=k)return"%"
s=b+1
if(!(s>=0&&s<k))return A.f(a,s)
r=a.charCodeAt(s)
if(!(l>=0))return A.f(a,l)
q=a.charCodeAt(l)
p=A.eZ(r)
o=A.eZ(q)
if(p<0||o<0)return"%"
n=p*16+o
if(n<127){if(!(n>=0))return A.f(m,n)
l=(m.charCodeAt(n)&1)!==0}else l=!1
if(l)return A.ax(c&&65<=n&&90>=n?(n|32)>>>0:n)
if(r>=97||q>=97)return B.a.l(a,b,b+3).toUpperCase()
return null},
fy(a){var s,r,q,p,o,n,m,l,k="0123456789ABCDEF"
if(a<=127){s=new Uint8Array(3)
s[0]=37
r=a>>>4
if(!(r<16))return A.f(k,r)
s[1]=k.charCodeAt(r)
s[2]=k.charCodeAt(a&15)}else{if(a>2047)if(a>65535){q=240
p=4}else{q=224
p=3}else{q=192
p=2}r=3*p
s=new Uint8Array(r)
for(o=0;--p,p>=0;q=128){n=B.b.bT(a,6*p)&63|q
if(!(o<r))return A.f(s,o)
s[o]=37
m=o+1
l=n>>>4
if(!(l<16))return A.f(k,l)
if(!(m<r))return A.f(s,m)
s[m]=k.charCodeAt(l)
l=o+2
if(!(l<r))return A.f(s,l)
s[l]=k.charCodeAt(n&15)
o+=3}}return A.ha(s,0,null)},
c0(a,b,c,d,e,f){var s=A.hy(a,b,c,d,e,f)
return s==null?B.a.l(a,b,c):s},
hy(a,b,c,d,e,f){var s,r,q,p,o,n,m,l,k,j,i=null,h=u.f
for(s=!e,r=a.length,q=b,p=q,o=i;q<c;){if(!(q>=0&&q<r))return A.f(a,q)
n=a.charCodeAt(q)
if(n<127&&(h.charCodeAt(n)&d)!==0)++q
else{m=1
if(n===37){l=A.fz(a,q,!1)
if(l==null){q+=3
continue}if("%"===l)l="%25"
else m=3}else if(n===92&&f)l="/"
else if(s&&n<=93&&(h.charCodeAt(n)&1024)!==0){A.b1(a,q,"Invalid character")
m=i
l=m}else{if((n&64512)===55296){k=q+1
if(k<c){if(!(k<r))return A.f(a,k)
j=a.charCodeAt(k)
if((j&64512)===56320){n=65536+((n&1023)<<10)+(j&1023)
m=2}}}l=A.fy(n)}if(o==null){o=new A.J("")
k=o}else k=o
k.a=(k.a+=B.a.l(a,p,q))+l
if(typeof m!=="number")return A.kB(m)
q+=m
p=q}}if(o==null)return i
if(p<c){s=B.a.l(a,p,c)
o.a+=s}s=o.a
return s.charCodeAt(0)==0?s:s},
hx(a){if(B.a.E(a,"."))return!0
return B.a.c8(a,"/.")!==-1},
jD(a){var s,r,q,p,o,n,m
if(!A.hx(a))return a
s=A.h([],t.s)
for(r=a.split("/"),q=r.length,p=!1,o=0;o<q;++o){n=r[o]
if(n===".."){m=s.length
if(m!==0){if(0>=m)return A.f(s,-1)
s.pop()
if(s.length===0)B.c.m(s,"")}p=!0}else{p="."===n
if(!p)B.c.m(s,n)}}if(p)B.c.m(s,"")
return B.c.aD(s,"/")},
jC(a,b){var s,r,q,p,o,n
if(!A.hx(a))return!b?A.hv(a):a
s=A.h([],t.s)
for(r=a.split("/"),q=r.length,p=!1,o=0;o<q;++o){n=r[o]
if(".."===n){if(s.length!==0&&B.c.gbk(s)!==".."){if(0>=s.length)return A.f(s,-1)
s.pop()}else B.c.m(s,"..")
p=!0}else{p="."===n
if(!p)B.c.m(s,n.length===0&&s.length===0?"./":n)}}if(s.length===0)return"./"
if(p)B.c.m(s,"")
if(!b){if(0>=s.length)return A.f(s,0)
B.c.p(s,0,A.hv(s[0]))}return B.c.aD(s,"/")},
hv(a){var s,r,q,p=u.f,o=a.length
if(o>=2&&A.hw(a.charCodeAt(0)))for(s=1;s<o;++s){r=a.charCodeAt(s)
if(r===58)return B.a.l(a,0,s)+"%3A"+B.a.aL(a,s+1)
if(r<=127){if(!(r<128))return A.f(p,r)
q=(p.charCodeAt(r)&8)===0}else q=!0
if(q)break}return a},
hw(a){var s=a|32
return 97<=s&&s<=122},
hd(a,b,c){var s,r,q,p,o,n,m,l,k="Invalid MIME type",j=A.h([b-1],t.t)
for(s=a.length,r=b,q=-1,p=null;r<s;++r){p=a.charCodeAt(r)
if(p===44||p===59)break
if(p===47){if(q<0){q=r
continue}throw A.i(A.M(k,a,r))}}if(q<0&&r>b)throw A.i(A.M(k,a,r))
while(p!==44){B.c.m(j,r);++r
for(o=-1;r<s;++r){if(!(r>=0))return A.f(a,r)
p=a.charCodeAt(r)
if(p===61){if(o<0)o=r}else if(p===59||p===44)break}if(o>=0)B.c.m(j,o)
else{n=B.c.gbk(j)
if(p!==44||r!==n+7||!B.a.u(a,"base64",n+1))throw A.i(A.M("Expecting '='",a,r))
break}}B.c.m(j,r)
m=r+1
if((j.length&1)===1)a=B.A.cg(a,m,s)
else{l=A.hy(a,m,s,256,!0,!1)
if(l!=null)a=B.a.U(a,m,s,l)}return new A.dF(a,j,c)},
hO(a,b,c,d,e){var s,r,q,p,o,n='\xe1\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\xe1\xe1\xe1\x01\xe1\xe1\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\xe1\xe3\xe1\xe1\x01\xe1\x01\xe1\xcd\x01\xe1\x01\x01\x01\x01\x01\x01\x01\x01\x0e\x03\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01"\x01\xe1\x01\xe1\xac\xe1\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\xe1\xe1\xe1\x01\xe1\xe1\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\xe1\xea\xe1\xe1\x01\xe1\x01\xe1\xcd\x01\xe1\x01\x01\x01\x01\x01\x01\x01\x01\x01\n\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01"\x01\xe1\x01\xe1\xac\xeb\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\xeb\xeb\xeb\x8b\xeb\xeb\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\xeb\x83\xeb\xeb\x8b\xeb\x8b\xeb\xcd\x8b\xeb\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x92\x83\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\x8b\xeb\x8b\xeb\x8b\xeb\xac\xeb\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\xeb\xeb\xeb\v\xeb\xeb\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\xebD\xeb\xeb\v\xeb\v\xeb\xcd\v\xeb\v\v\v\v\v\v\v\v\x12D\v\v\v\v\v\v\v\v\v\v\xeb\v\xeb\v\xeb\xac\xe5\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\xe5\xe5\xe5\x05\xe5D\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe8\x8a\xe5\xe5\x05\xe5\x05\xe5\xcd\x05\xe5\x05\x05\x05\x05\x05\x05\x05\x05\x05\x8a\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05f\x05\xe5\x05\xe5\xac\xe5\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05\xe5\xe5\xe5\x05\xe5D\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\xe5\x8a\xe5\xe5\x05\xe5\x05\xe5\xcd\x05\xe5\x05\x05\x05\x05\x05\x05\x05\x05\x05\x8a\x05\x05\x05\x05\x05\x05\x05\x05\x05\x05f\x05\xe5\x05\xe5\xac\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7D\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\x8a\xe7\xe7\xe7\xe7\xe7\xe7\xcd\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\x8a\xe7\x07\x07\x07\x07\x07\x07\x07\x07\x07\xe7\xe7\xe7\xe7\xe7\xac\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7D\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\x8a\xe7\xe7\xe7\xe7\xe7\xe7\xcd\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\xe7\x8a\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\xe7\xe7\xe7\xe7\xe7\xac\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\x05\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\xeb\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\xeb\xeb\xeb\v\xeb\xeb\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\xeb\xea\xeb\xeb\v\xeb\v\xeb\xcd\v\xeb\v\v\v\v\v\v\v\v\x10\xea\v\v\v\v\v\v\v\v\v\v\xeb\v\xeb\v\xeb\xac\xeb\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\xeb\xeb\xeb\v\xeb\xeb\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\xeb\xea\xeb\xeb\v\xeb\v\xeb\xcd\v\xeb\v\v\v\v\v\v\v\v\x12\n\v\v\v\v\v\v\v\v\v\v\xeb\v\xeb\v\xeb\xac\xeb\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\xeb\xeb\xeb\v\xeb\xeb\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\xeb\xea\xeb\xeb\v\xeb\v\xeb\xcd\v\xeb\v\v\v\v\v\v\v\v\v\n\v\v\v\v\v\v\v\v\v\v\xeb\v\xeb\v\xeb\xac\xec\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\xec\xec\xec\f\xec\xec\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\f\xec\xec\xec\xec\f\xec\f\xec\xcd\f\xec\f\f\f\f\f\f\f\f\f\xec\f\f\f\f\f\f\f\f\f\f\xec\f\xec\f\xec\f\xed\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\xed\xed\xed\r\xed\xed\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\xed\xed\xed\xed\r\xed\r\xed\xed\r\xed\r\r\r\r\r\r\r\r\r\xed\r\r\r\r\r\r\r\r\r\r\xed\r\xed\r\xed\r\xe1\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\xe1\xe1\xe1\x01\xe1\xe1\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\xe1\xea\xe1\xe1\x01\xe1\x01\xe1\xcd\x01\xe1\x01\x01\x01\x01\x01\x01\x01\x01\x0f\xea\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01"\x01\xe1\x01\xe1\xac\xe1\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\xe1\xe1\xe1\x01\xe1\xe1\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\xe1\xe9\xe1\xe1\x01\xe1\x01\xe1\xcd\x01\xe1\x01\x01\x01\x01\x01\x01\x01\x01\x01\t\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01"\x01\xe1\x01\xe1\xac\xeb\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\xeb\xeb\xeb\v\xeb\xeb\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\xeb\xea\xeb\xeb\v\xeb\v\xeb\xcd\v\xeb\v\v\v\v\v\v\v\v\x11\xea\v\v\v\v\v\v\v\v\v\v\xeb\v\xeb\v\xeb\xac\xeb\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\xeb\xeb\xeb\v\xeb\xeb\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\xeb\xe9\xeb\xeb\v\xeb\v\xeb\xcd\v\xeb\v\v\v\v\v\v\v\v\v\t\v\v\v\v\v\v\v\v\v\v\xeb\v\xeb\v\xeb\xac\xeb\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\xeb\xeb\xeb\v\xeb\xeb\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\xeb\xea\xeb\xeb\v\xeb\v\xeb\xcd\v\xeb\v\v\v\v\v\v\v\v\x13\xea\v\v\v\v\v\v\v\v\v\v\xeb\v\xeb\v\xeb\xac\xeb\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\xeb\xeb\xeb\v\xeb\xeb\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\v\xeb\xea\xeb\xeb\v\xeb\v\xeb\xcd\v\xeb\v\v\v\v\v\v\v\v\v\xea\v\v\v\v\v\v\v\v\v\v\xeb\v\xeb\v\xeb\xac\xf5\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\xf5\x15\xf5\x15\x15\xf5\x15\x15\x15\x15\x15\x15\x15\x15\x15\x15\xf5\xf5\xf5\xf5\xf5\xf5'
for(s=a.length,r=b;r<c;++r){if(!(r<s))return A.f(a,r)
q=a.charCodeAt(r)^96
if(q>95)q=31
p=d*96+q
if(!(p<2112))return A.f(n,p)
o=n.charCodeAt(p)
d=o&31
B.c.p(e,o>>>5,r)}return d},
ds:function ds(a,b){this.a=a
this.b=b},
eq:function eq(){},
u:function u(){},
ca:function ca(a){this.a=a},
ad:function ad(){},
a3:function a3(a,b,c,d){var _=this
_.a=a
_.b=b
_.c=c
_.d=d},
bx:function bx(a,b,c,d,e,f){var _=this
_.e=a
_.f=b
_.a=c
_.b=d
_.c=e
_.d=f},
cn:function cn(a,b,c,d,e){var _=this
_.f=a
_.a=b
_.b=c
_.c=d
_.d=e},
cD:function cD(a,b,c,d){var _=this
_.a=a
_.b=b
_.c=c
_.d=d},
bE:function bE(a){this.a=a},
cN:function cN(a){this.a=a},
aX:function aX(a){this.a=a},
ci:function ci(a){this.a=a},
cE:function cE(){},
bA:function bA(){},
es:function es(a){this.a=a},
a8:function a8(a,b,c){this.a=a
this.b=b
this.c=c},
v:function v(){},
p:function p(){},
d4:function d4(){},
J:function J(a){this.a=a},
dG:function dG(a){this.a=a},
c_:function c_(a,b,c,d,e,f,g){var _=this
_.a=a
_.b=b
_.c=c
_.d=d
_.e=e
_.f=f
_.r=g
_.y=_.w=$},
dF:function dF(a,b,c){this.a=a
this.b=b
this.c=c},
d2:function d2(a,b,c,d,e,f,g,h){var _=this
_.a=a
_.b=b
_.c=c
_.d=d
_.e=e
_.f=f
_.r=g
_.w=h
_.x=null},
cY:function cY(a,b,c,d,e,f,g){var _=this
_.a=a
_.b=b
_.c=c
_.d=d
_.e=e
_.f=f
_.r=g
_.y=_.w=$},
iG(a){return A.j(a)},
dt:function dt(a){this.a=a},
c2(a){var s
if(typeof a=="function")throw A.i(A.Y("Attempting to rewrap a JS function.",null))
s=function(b,c){return function(d){return b(c,d,arguments.length)}}(A.jN,a)
s[$.bb()]=a
return s},
hH(a){var s
if(typeof a=="function")throw A.i(A.Y("Attempting to rewrap a JS function.",null))
s=function(b,c){return function(d,e,f){return b(c,d,e,f,arguments.length)}}(A.jP,a)
s[$.bb()]=a
return s},
c3(a){var s
if(typeof a=="function")throw A.i(A.Y("Attempting to rewrap a JS function.",null))
s=function(b,c){return function(d,e,f,g){return b(c,d,e,f,g,arguments.length)}}(A.jQ,a)
s[$.bb()]=a
return s},
d8(a,b){var s
if(typeof a=="function")throw A.i(A.Y("Attempting to rewrap a JS function.",null))
s=function(c,d,e){return function(){return c(d,Array.prototype.slice.call(arguments,0,Math.min(arguments.length,e)))}}(A.jS,a,b)
s[$.bb()]=a
return s},
jN(a,b,c){t.Z.a(a)
if(A.j(c)>=1)return a.$1(b)
return a.$0()},
jO(a,b,c,d){t.Z.a(a)
A.j(d)
if(d>=2)return a.$2(b,c)
if(d===1)return a.$1(b)
return a.$0()},
jP(a,b,c,d,e){t.Z.a(a)
A.j(e)
if(e>=3)return a.$3(b,c,d)
if(e===2)return a.$2(b,c)
if(e===1)return a.$1(b)
return a.$0()},
jQ(a,b,c,d,e,f){t.Z.a(a)
A.j(f)
if(f>=4)return a.$4(b,c,d,e)
if(f===3)return a.$3(b,c,d)
if(f===2)return a.$2(b,c)
if(f===1)return a.$1(b)
return a.$0()},
jR(a,b,c,d,e,f,g){t.Z.a(a)
A.j(g)
if(g>=5)return a.$5(b,c,d,e,f)
if(g===4)return a.$4(b,c,d,e)
if(g===3)return a.$3(b,c,d)
if(g===2)return a.$2(b,c)
if(g===1)return a.$1(b)
return a.$0()},
jS(a,b){t.Z.a(a)
t.j.a(b)
return A.iN(a,b,null)},
dc(a,b,c,d){return d.a(a[b].apply(a,c))},
fM(a,b){var s=new A.E($.y,b.i("E<0>")),r=new A.bJ(s,b.i("bJ<0>"))
a.then(A.c7(new A.f7(r,b),1),A.c7(new A.f8(r),1))
return s},
f7:function f7(a,b){this.a=a
this.b=b},
f8:function f8(a){this.a=a},
eD:function eD(){},
bQ:function bQ(){this.b=this.a=0},
V:function V(a,b,c){this.a=a
this.b=b
this.c=c},
ce:function ce(a,b,c){this.a=a
this.b=b
this.c=c},
cl:function cl(){},
K(a,b,c,d,e,f,g,h){var s,r
if(f==null)s=new A.e(new Float32Array(2))
else s=f
if(e==null)r=new A.e(new Float32Array(2))
else r=e
return new A.dh(h,s,g,r,a,b,c,d)},
P(a,b,c){var s=b==null?new A.H(0.6,0,0):b
return new A.dz(s,a,new A.cm(),c)},
fW(a,b){var s=A.h([new A.H(0.6,0,0)],t.J)
return new A.di(b,s,new A.cm(),a)},
cm:function cm(){},
H:function H(a,b,c){this.a=a
this.b=b
this.f=c},
e2:function e2(a){this.a=a},
dh:function dh(a,b,c,d,e,f,g,h){var _=this
_.a=a
_.b=b
_.c=c
_.d=d
_.e=e
_.Q=f
_.at=g
_.ax=h},
dz:function dz(a,b,c,d){var _=this
_.a=a
_.b=b
_.c=c
_.d=d},
di:function di(a,b,c,d){var _=this
_.a=a
_.b=b
_.c=c
_.e=d},
aK:function aK(a,b){this.a=a
this.b=b},
am:function am(a,b){this.a=a
this.b=b},
Z:function Z(a,b){this.a=a
this.b=b},
bw(a,b,c){return new A.a_(null,c,a,b)},
ay:function ay(){},
W:function W(a,b){this.a=a
this.b=b},
bd:function bd(a,b,c){this.a=a
this.b=b
this.c=c},
aV:function aV(a,b){this.a=a
this.b=b},
a_:function a_(a,b,c,d){var _=this
_.a=a
_.b=b
_.c=c
_.d=d},
aM:function aM(a,b,c,d,e,f,g,h,i,j){var _=this
_.e=a
_.f=b
_.r=c
_.w=d
_.x=e
_.y=f
_.a=g
_.b=h
_.c=i
_.d=j},
dj:function dj(a,b,c){this.a=a
this.b=b
this.c=c},
au:function au(){},
bo:function bo(){},
dr:function dr(a,b,c,d,e,f,g,h){var _=this
_.e=a
_.f=b
_.r=c
_.w=d
_.a=e
_.b=f
_.c=g
_.d=h},
dq:function dq(a,b,c){this.a=a
this.b=b
this.c=c},
h8(a,b,c,d){return new A.cJ(c,d,a,b,!1,null)},
cJ:function cJ(a,b,c,d,e,f){var _=this
_.e=a
_.f=b
_.a=c
_.b=d
_.c=e
_.d=f},
dx:function dx(a,b,c){this.a=a
this.b=b
this.c=c},
e1:function e1(a,b,c,d,e,f,g,h,i,j){var _=this
_.e=a
_.f=b
_.r=c
_.x=d
_.at=e
_.ax=f
_.a=g
_.b=h
_.c=i
_.d=j},
cT:function cT(a,b,c){this.a=a
this.b=b
this.c=c},
al:function al(a,b){this.a=a
this.b=b},
aZ:function aZ(a,b){this.a=a
this.b=b},
c8:function c8(a,b){this.a=a
this.b=b},
aW:function aW(a,b,c){this.a=a
this.b=b
this.c=c},
bI:function bI(a,b,c,d,e,f,g){var _=this
_.a=a
_.b=b
_.c=c
_.d=d
_.e=e
_.f=f
_.r=!1
_.w=g},
ec:function ec(){},
e5:function e5(a,b){this.a=a
this.b=b},
e9:function e9(a,b){this.a=a
this.b=b},
e3:function e3(a){this.a=a},
e8:function e8(a){this.a=a},
e7:function e7(a){this.a=a},
e6:function e6(a){this.a=a},
eb:function eb(a){this.a=a},
e4:function e4(a){this.a=a},
ea:function ea(a){this.a=a},
f2(a){var s=0,r=A.d9(t.H),q,p,o,n
var $async$f2=A.db(function(b,c){if(b===1)return A.d5(c,r)
for(;;)switch(s){case 0:if($.a6!=null){s=1
break}p=t.D
o=A.h([],p)
B.c.S(o,A.h([A.cQ("packages/forge2d/src/backend/wasm/box2d.wasm"),A.cQ("/packages/forge2d/src/backend/wasm/box2d.wasm"),A.cQ("assets/packages/forge2d/lib/src/backend/wasm/box2d.wasm"),A.cQ("box2d.wasm")],p))
n=$
s=3
return A.b4(A.bH(o),$async$f2)
case 3:n.a6=c
case 1:return A.d6(q,r)}})
return A.d7($async$f2,r)},
m(){var s,r=$.a6
if(r==null)throw A.i(A.C(u.j))
s=t.S
return new A.a0(r,A.B(s,t.p),A.B(s,t._))},
by(a){var s=4294967296
if(a===-1)return B.a0
if(a<0)return new A.N(a>>>0,B.b.R(a,32))
return new A.N(B.b.G(a,s),B.b.j(a,s))},
a0:function a0(a,b,c){this.a=a
this.b=b
this.c=c},
dw:function dw(a){this.a=a},
bH(a){var s=0,r=A.d9(t.B),q,p=2,o=[],n,m,l,k,j,i,h,g,f,e,d,c,b
var $async$bH=A.db(function(a0,a1){if(a0===1){o.push(a1)
s=p}for(;;)switch(s){case 0:c=null
j=a.length,i=t.h,h=v.G,g=t.m,f=0
case 3:if(!(f<a.length)){s=5
break}n=a[f]
p=7
s=10
return A.b4(A.fM(A.t(h.fetch(J.bc(n))),g),$async$bH)
case 10:m=a1
if(!A.hD(m.ok)){c=new A.aX("HTTP error fetching "+A.r(n))
s=4
break}s=11
return A.b4(A.fM(A.t(m.arrayBuffer()),i),$async$bH)
case 11:l=a1
s=12
return A.b4(A.dJ(l),$async$bH)
case 12:e=a1
q=e
s=1
break
p=2
s=9
break
case 7:p=6
b=o.pop()
k=A.aJ(b)
c=k
s=9
break
case 6:s=2
break
case 9:case 4:a.length===j||(0,A.S)(a),++f
s=3
break
case 5:throw A.i(A.C("Could not load box2d.wasm from any of: "+B.c.aD(a,", ")+". Pass the location explicitly: initializeForge2D(wasmUri: ...). Last error: "+A.r(c)))
case 1:return A.d6(q,r)
case 2:return A.d5(o.at(-1),r)}})
return A.d7($async$bH,r)},
dJ(a){var s=0,r=A.d9(t.B),q,p,o,n,m,l,k,j,i,h,g,f
var $async$dJ=A.db(function(b,c){if(b===1)return A.d5(c,r)
for(;;)switch(s){case 0:j=A.j8("runtime")
i={}
h=new A.e0(i)
h.$2("f2d_host_cast_ray",A.d8(new A.dK(j),7))
p=new A.dL(j)
if(typeof p=="function")A.A(A.Y("Attempting to rewrap a JS function.",null))
o=function(d,e){return function(a0,a1){return d(e,a0,a1,arguments.length)}}(A.jO,p)
n=$.bb()
o[n]=p
h.$2("f2d_host_overlap",o)
h.$2("f2d_host_custom_filter",A.c3(new A.dM(j)))
h.$2("f2d_host_pre_solve",A.d8(new A.dT(j),6))
h.$2("f2d_host_draw_polygon",A.hH(new A.dU(j)))
h.$2("f2d_host_draw_solid_polygon",A.d8(new A.dV(j),8))
h.$2("f2d_host_draw_circle",A.c3(new A.dW(j)))
h.$2("f2d_host_draw_solid_circle",A.d8(new A.dX(j),6))
h.$2("f2d_host_draw_solid_capsule",A.d8(new A.dY(j),6))
p=new A.dZ(j)
if(typeof p=="function")A.A(A.Y("Attempting to rewrap a JS function.",null))
o=function(d,e){return function(a0,a1,a2,a3,a4){return d(e,a0,a1,a2,a3,a4,arguments.length)}}(A.jR,p)
o[n]=p
h.$2("f2d_host_draw_segment",o)
h.$2("f2d_host_draw_transform",A.c3(new A.e_(j)))
h.$2("f2d_host_draw_point",A.c3(new A.dN(j)))
h.$2("f2d_host_draw_string",A.c3(new A.dO(j)))
h.$2("emscripten_notify_memory_growth",A.c2(new A.dP(j)))
m={}
m.clock_time_get=A.hH(new A.dQ(j))
m.fd_write=A.c3(new A.dR(j))
m.proc_exit=A.c2(new A.dS())
l={}
l.env=i
l.wasi_snapshot_preview1=m
g=A
f=A
s=3
return A.b4(A.fM(A.t(v.G.WebAssembly.instantiate(a,l)),t.m),$async$dJ)
case 3:k=g.t(f.t(c.instance).exports)
h=k.memory
h.toString
h=new A.cS(k,A.t(h),new A.dI(),A.B(t.N,t.g))
h.am()
p=h.aS(4096)
h.x!==$&&A.kL("_scratch")
h.x=p
q=j.b=h
s=1
break
case 1:return A.d6(q,r)}})
return A.d7($async$dJ,r)},
dI:function dI(){var _=this
_.e=_.d=_.c=_.b=null},
cR:function cR(a,b,c,d,e,f,g,h,i){var _=this
_.a=a
_.b=b
_.c=c
_.d=d
_.e=e
_.f=f
_.r=g
_.w=h
_.x=i},
cS:function cS(a,b,c,d){var _=this
_.a=a
_.b=b
_.c=c
_.r=_.f=_.d=$
_.w=0
_.x=$
_.as=_.Q=_.z=_.y=0
_.at=d},
e0:function e0(a){this.a=a},
dK:function dK(a){this.a=a},
dL:function dL(a){this.a=a},
dM:function dM(a){this.a=a},
dT:function dT(a){this.a=a},
dU:function dU(a){this.a=a},
dV:function dV(a){this.a=a},
dW:function dW(a){this.a=a},
dX:function dX(a){this.a=a},
dY:function dY(a){this.a=a},
dZ:function dZ(a){this.a=a},
e_:function e_(a){this.a=a},
dN:function dN(a){this.a=a},
dO:function dO(a){this.a=a},
dP:function dP(a){this.a=a},
dQ:function dQ(a){this.a=a},
dR:function dR(a){this.a=a},
dS:function dS(){},
e:function e(a){this.a=a},
hj(a,b,c,d,e){var s=A.kq(new A.er(c),t.m)
s=s==null?null:A.c2(s)
if(s!=null)a.addEventListener(b,s,!1)
return new A.bL(a,b,s,!1,e.i("bL<0>"))},
kq(a,b){var s=$.y
if(s===B.i)return a
return s.bX(a,b)},
fp:function fp(a){this.$ti=a},
bK:function bK(){},
cZ:function cZ(a,b,c,d){var _=this
_.a=a
_.b=b
_.c=c
_.$ti=d},
bL:function bL(a,b,c,d,e){var _=this
_.b=a
_.c=b
_.d=c
_.e=d
_.$ti=e},
er:function er(a){this.a=a},
cd:function cd(a,b){var _=this
_.ax=a
_.ay=b
_.ch=40
_.CW=20
_.cy=_.cx=0
_.b=!0},
f5(){var s=0,r=A.d9(t.H),q,p,o,n
var $async$f5=A.db(function(a,b){if(a===1)return A.d5(b,r)
for(;;)switch(s){case 0:s=2
return A.b4(A.f3(),$async$f5)
case 2:q=v.G
p=A.aD(A.t(q.document).getElementById("loading"))
if(p!=null)p.remove()
p=B.c.gbe($.de())
o=A.aD(A.t(q.document).getElementById("canvas"))
o.toString
n=A.aD(A.t(q.document).getElementById("hint"))
n.toString
q=A.aD(A.t(q.document).getElementById("nav"))
q.toString
p=new A.cU(o,n,q,new A.aU(),p)
o=A.aD(o.getContext("2d"))
o.toString
p.d=new A.cd(o,new A.e(new Float32Array(2)))
p.by()
return A.d6(null,r)}})
return A.d7($async$f5,r)},
b5(a,b,c,d){return b.addEventListener(a.b.toLowerCase(),A.c2(new A.eU(c,d)))},
a7:function a7(a,b){this.a=a
this.b=b},
eU:function eU(a,b){this.a=a
this.b=b},
cU:function cU(a,b,c,d,e){var _=this
_.a=a
_.b=b
_.c=c
_.d=$
_.e=null
_.f=d
_.r=e
_.w=null
_.y=_.x=0},
ed:function ed(a,b){this.a=a
this.b=b},
ee:function ee(a){this.a=a},
ef:function ef(a){this.a=a},
eg:function eg(a){this.a=a},
eh:function eh(a){this.a=a},
ei:function ei(a){this.a=a},
ej:function ej(a,b){this.a=a
this.b=b},
ek:function ek(){},
hQ(a,b,c){var s=b==null?0.6:b
return A.P(1,new A.H(s,c==null?0:c,a),a)},
aT(a,b,c,d,e){return new A.ab(d,c,e,b,a)},
eT(a,b){var s=new A.e(new Float32Array(2))
s.k(0,-1)
s=a.v(A.K(0,!0,!1,!1,null,s,B.d,B.j))
s.C(A.bw(b,1,0),A.P(1,new A.H(0.6,0,6583435),6583435))
return s},
aU:function aU(){var _=this
_.d=_.c=_.b=_.a=null},
ab:function ab(a,b,c,d,e){var _=this
_.a=a
_.b=b
_.c=c
_.d=d
_.e=e},
fe:function fe(){},
ff:function ff(){},
fl:function fl(a,b){this.a=a
this.b=b},
fd:function fd(a,b,c,d,e){var _=this
_.a=a
_.b=b
_.c=c
_.d=d
_.e=e},
fg:function fg(){},
fc:function fc(a){this.a=a},
fh:function fh(){},
fi:function fi(){},
fm:function fm(){},
fj:function fj(){},
fk:function fk(){},
fn:function fn(a){this.a=a},
f9:function f9(a){this.a=a},
fa:function fa(a){this.a=a},
fb:function fb(a,b){this.a=a
this.b=b},
f3(){var s=0,r=A.d9(t.H)
var $async$f3=A.db(function(a,b){if(a===1)return A.d5(b,r)
for(;;)switch(s){case 0:s=2
return A.b4(A.f2(null),$async$f3)
case 2:if($.d==null)$.d=A.m()
return A.d6(null,r)}})
return A.d7($async$f3,r)}},B={}
var w=[A,J,B]
var $={}
A.fr.prototype={}
J.co.prototype={
B(a,b){return a===b},
gn(a){return A.cG(a)},
h(a){return"Instance of '"+A.cH(a)+"'"},
bl(a,b){throw A.i(A.h1(a,t.A.a(b)))},
gt(a){return A.aF(A.fB(this))}}
J.cq.prototype={
h(a){return String(a)},
gn(a){return a?519018:218159},
gt(a){return A.aF(t.y)},
$io:1,
$iq:1}
J.bj.prototype={
B(a,b){return null==b},
h(a){return"null"},
gn(a){return 0},
$io:1}
J.bm.prototype={$iz:1}
J.aj.prototype={
gn(a){return 0},
h(a){return String(a)}}
J.cF.prototype={}
J.bC.prototype={}
J.a4.prototype={
h(a){var s=a[$.i6()]
if(s==null)s=a[$.bb()]
if(s==null)return this.bA(a)
return"JavaScript function for "+J.bc(s)},
$ias:1}
J.bl.prototype={
gn(a){return 0},
h(a){return String(a)}}
J.bn.prototype={
gn(a){return 0},
h(a){return String(a)}}
J.n.prototype={
m(a,b){A.b2(a).c.a(b)
a.$flags&1&&A.ar(a,29)
a.push(b)},
S(a,b){A.b2(a).i("w<1>").a(b)
a.$flags&1&&A.ar(a,"addAll",2)
this.bC(a,b)
return},
bC(a,b){var s,r
t.b.a(b)
s=b.length
if(s===0)return
if(a===b)throw A.i(A.cj(a))
for(r=0;r<s;++r)a.push(b[r])},
M(a){a.$flags&1&&A.ar(a,"clear","clear")
a.length=0},
aD(a,b){var s,r=A.h0(a.length,"",!1,t.N)
for(s=0;s<a.length;++s)this.p(r,s,A.r(a[s]))
return r.join(b)},
c6(a,b,c){var s,r,q,p=A.b2(a)
p.i("q(1)").a(b)
p.i("1()?").a(c)
s=a.length
for(r=0;r<s;++r){q=a[r]
if(b.$1(q))return q
if(a.length!==s)throw A.i(A.cj(a))}p=c.$0()
return p},
gbe(a){if(a.length>0)return a[0]
throw A.i(A.fZ())},
gbk(a){var s=a.length
if(s>0)return a[s-1]
throw A.i(A.fZ())},
h(a){return A.h_(a,"[","]")},
gbj(a){return new J.c9(a,a.length,A.b2(a).i("c9<1>"))},
gn(a){return A.cG(a)},
gA(a){return a.length},
p(a,b,c){A.b2(a).c.a(c)
a.$flags&2&&A.ar(a)
if(!(b>=0&&b<a.length))throw A.i(A.hU(a,b))
a[b]=c},
$iw:1,
$ik:1}
J.cp.prototype={
cp(a){var s,r,q
if(!Array.isArray(a))return null
s=a.$flags|0
if((s&4)!==0)r="const, "
else if((s&2)!==0)r="unmodifiable, "
else r=(s&1)!==0?"fixed, ":""
q="Instance of '"+A.cH(a)+"'"
if(r==="")return q
return q+" ("+r+"length: "+a.length+")"}}
J.dk.prototype={}
J.c9.prototype={
gZ(){var s=this.d
return s==null?this.$ti.c.a(s):s},
a_(){var s,r=this,q=r.a,p=q.length
if(r.b!==p){q=A.S(q)
throw A.i(q)}s=r.c
if(s>=p){r.d=null
return!1}r.d=q[s]
r.c=s+1
return!0}}
J.bk.prototype={
aq(a,b){var s
if(a<b)return-1
else if(a>b)return 1
else if(a===b){if(a===0){s=this.gaC(b)
if(this.gaC(a)===s)return 0
if(this.gaC(a))return-1
return 1}return 0}else if(isNaN(a)){if(isNaN(b))return 0
return 1}else return-1},
gaC(a){return a===0?1/a<0:a<0},
W(a){var s
if(a>=-2147483648&&a<=2147483647)return a|0
if(isFinite(a)){s=a<0?Math.ceil(a):Math.floor(a)
return s+0}throw A.i(A.bF(""+a+".toInt()"))},
bf(a){var s,r
if(a>=0){if(a<=2147483647)return a|0}else if(a>=-2147483648){s=a|0
return a===s?s:s-1}r=Math.floor(a)
if(isFinite(r))return r
throw A.i(A.bF(""+a+".floor()"))},
br(a){if(a>0){if(a!==1/0)return Math.round(a)}else if(a>-1/0)return 0-Math.round(0-a)
throw A.i(A.bF(""+a+".round()"))},
b3(a,b,c){if(B.b.aq(b,c)>0)throw A.i(A.fG(b))
if(this.aq(a,b)<0)return b
if(this.aq(a,c)>0)return c
return a},
h(a){if(a===0&&1/a<0)return"-0.0"
else return""+a},
gn(a){var s,r,q,p,o=a|0
if(a===o)return o&536870911
s=Math.abs(a)
r=Math.log(s)/0.6931471805599453|0
q=Math.pow(2,r)
p=s<1?s/q:q/s
return((p*9007199254740992|0)+(p*3542243181176521|0))*599197+r*1259&536870911},
G(a,b){var s=a%b
if(s===0)return 0
if(s>0)return s
return s+b},
j(a,b){return(a|0)===a?a/b|0:this.bU(a,b)},
bU(a,b){var s=a/b
if(s>=-2147483648&&s<=2147483647)return s|0
if(s>0){if(s!==1/0)return Math.floor(s)}else if(s>-1/0)return Math.ceil(s)
throw A.i(A.bF("Result of truncating division is "+A.r(s)+": "+A.r(a)+" ~/ "+b))},
R(a,b){var s
if(a>0)s=this.aV(a,b)
else{s=b>31?31:b
s=a>>s>>>0}return s},
bT(a,b){if(0>b)throw A.i(A.fG(b))
return this.aV(a,b)},
aV(a,b){return b>31?0:a>>>b},
gt(a){return A.aF(t.q)},
$ia:1,
$iah:1}
J.bi.prototype={
gt(a){return A.aF(t.S)},
$io:1,
$ib:1}
J.cs.prototype={
gt(a){return A.aF(t.i)},
$io:1}
J.aO.prototype={
U(a,b,c,d){var s=A.cI(b,c,a.length)
return a.substring(0,b)+d+a.substring(s)},
u(a,b,c){var s
if(c<0||c>a.length)throw A.i(A.aa(c,0,a.length,null,null))
s=c+b.length
if(s>a.length)return!1
return b===a.substring(c,s)},
E(a,b){return this.u(a,b,0)},
l(a,b,c){return a.substring(b,A.cI(b,c,a.length))},
aL(a,b){return this.l(a,b,null)},
bw(a,b){var s,r
if(0>=b)return""
if(b===1||a.length===0)return a
if(b!==b>>>0)throw A.i(B.H)
for(s=a,r="";;){if((b&1)===1)r=s+r
b=b>>>1
if(b===0)break
s+=s}return r},
a6(a,b,c){var s
if(c<0||c>a.length)throw A.i(A.aa(c,0,a.length,null,null))
s=a.indexOf(b,c)
return s},
c8(a,b){return this.a6(a,b,0)},
h(a){return a},
gn(a){var s,r,q
for(s=a.length,r=0,q=0;q<s;++q){r=r+a.charCodeAt(q)&536870911
r=r+((r&524287)<<10)&536870911
r^=r>>6}r=r+((r&67108863)<<3)&536870911
r^=r>>11
return r+((r&16383)<<15)&536870911},
gt(a){return A.aF(t.N)},
gA(a){return a.length},
$io:1,
$ih3:1,
$iD:1}
A.aw.prototype={
h(a){return"LateInitializationError: "+this.a}}
A.dy.prototype={}
A.cu.prototype={
gZ(){var s=this.d
return s==null?this.$ti.c.a(s):s},
a_(){var s,r=this,q=r.a,p=J.hV(q),o=p.gA(q)
if(r.b!==o)throw A.i(A.cj(q))
s=r.c
if(s>=o){r.d=null
return!1}r.d=p.c4(q,s);++r.c
return!0}}
A.L.prototype={}
A.ao.prototype={
gn(a){var s=this._hashCode
if(s!=null)return s
s=664597*B.a.gn(this.a)&536870911
this._hashCode=s
return s},
h(a){return'Symbol("'+this.a+'")'},
B(a,b){if(b==null)return!1
return b instanceof A.ao&&this.a===b.a},
$iaY:1}
A.bR.prototype={$r:"+categoryBits,customColor,density,enableContactEvents,enableHitEvents,enablePreSolveEvents,enableSensorEvents,friction,groupIndex,invokeContactCreation,isSensor,maskBits,restitution,rollingResistance,tangentSpeed,updateBodyMass,userMaterialId(1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17)",$s:6}
A.N.prototype={$r:"+(1,2)",$s:1}
A.bS.prototype={$r:"+drawBodyNames,drawBounds,drawCircle,drawContactFeatures,drawContactImpulses,drawContactNormals,drawContacts,drawFrictionImpulses,drawGraphColors,drawIslands,drawJointExtras,drawJoints,drawMass,drawPoint,drawPolygon,drawSegment,drawShapes,drawSolidCapsule,drawSolidCircle,drawSolidPolygon,drawString,drawTransform,drawingBounds(1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23)",$s:7}
A.bf.prototype={}
A.be.prototype={
h(a){return A.dn(this)},
$ia9:1}
A.bg.prototype={
gA(a){return this.b.length},
T(a,b){var s,r,q,p,o=this
o.$ti.i("~(1,2)").a(b)
s=o.$keys
if(s==null){s=Object.keys(o.a)
o.$keys=s}s=s
r=o.b
for(q=s.length,p=0;p<q;++p)b.$2(s[p],r[p])}}
A.cr.prototype={
gcd(){var s=this.a
if(s instanceof A.ao)return s
return this.a=new A.ao(A.Q(s))},
gcj(){var s,r,q,p,o=this
if(o.c===1)return B.v
s=o.d
r=s.length-o.e.length-o.f
if(r===0)return B.v
q=[]
for(p=0;p<r;++p){if(!(p<s.length))return A.f(s,p)
q.push(s[p])}q.$flags=3
return q},
gce(){var s,r,q,p,o,n,m,l,k=this
if(k.c!==0)return B.x
s=k.e
r=s.length
q=k.d
p=q.length-r-k.f
if(r===0)return B.x
o=new A.av(t.bV)
for(n=0;n<r;++n){if(!(n<s.length))return A.f(s,n)
m=A.Q(s[n])
l=p+n
if(!(l>=0&&l<q.length))return A.f(q,l)
o.p(0,new A.ao(m),q[l])}return new A.bf(o,t.k)},
$ifY:1}
A.dv.prototype={
$2(a,b){var s
A.Q(a)
s=this.a
s.b=s.b+"$"+a
B.c.m(this.b,a)
B.c.m(this.c,b);++s.a},
$S:16}
A.bz.prototype={}
A.dD.prototype={
J(a){var s,r,q=this,p=new RegExp(q.a).exec(a)
if(p==null)return null
s=Object.create(null)
r=q.b
if(r!==-1)s.arguments=p[r+1]
r=q.c
if(r!==-1)s.argumentsExpr=p[r+1]
r=q.d
if(r!==-1)s.expr=p[r+1]
r=q.e
if(r!==-1)s.method=p[r+1]
r=q.f
if(r!==-1)s.receiver=p[r+1]
return s}}
A.bv.prototype={
h(a){return"Null check operator used on a null value"}}
A.ct.prototype={
h(a){var s,r=this,q="NoSuchMethodError: method not found: '",p=r.b
if(p==null)return"NoSuchMethodError: "+r.a
s=r.c
if(s==null)return q+p+"' ("+r.a+")"
return q+p+"' on '"+s+"' ("+r.a+")"}}
A.cO.prototype={
h(a){var s=this.a
return s.length===0?"Error":"Error: "+s}}
A.du.prototype={
h(a){return"Throw of null ('"+(this.a===null?"null":"undefined")+"' from JavaScript)"}}
A.bh.prototype={}
A.bT.prototype={
h(a){var s,r=this.b
if(r!=null)return r
r=this.a
s=r!==null&&typeof r==="object"?r.stack:null
return this.b=s==null?"":s},
$ian:1}
A.ai.prototype={
h(a){var s=this.constructor,r=s==null?null:s.name
return"Closure '"+A.i5(r==null?"unknown":r)+"'"},
$ias:1,
gcv(){return this},
$C:"$1",
$R:1,
$D:null}
A.cf.prototype={$C:"$0",$R:0}
A.cg.prototype={$C:"$2",$R:2}
A.cM.prototype={}
A.cL.prototype={
h(a){var s=this.$static_name
if(s==null)return"Closure of unknown static method"
return"Closure '"+A.i5(s)+"'"}}
A.aL.prototype={
B(a,b){if(b==null)return!1
if(this===b)return!0
if(!(b instanceof A.aL))return!1
return this.$_target===b.$_target&&this.a===b.a},
gn(a){return(A.hZ(this.a)^A.cG(this.$_target))>>>0},
h(a){return"Closure '"+this.$_name+"' of "+("Instance of '"+A.cH(this.a)+"'")}}
A.cK.prototype={
h(a){return"RuntimeError: "+this.a}}
A.eG.prototype={}
A.av.prototype={
gA(a){return this.a},
bZ(a){var s=this.b
if(s==null)return!1
return s[a]!=null},
O(a,b){var s,r,q,p,o=null
if(typeof b=="string"){s=this.b
if(s==null)return o
r=s[b]
q=r==null?o:r.b
return q}else if(typeof b=="number"&&(b&0x3fffffff)===b){p=this.c
if(p==null)return o
r=p[b]
q=r==null?o:r.b
return q}else return this.c9(b)},
c9(a){var s,r,q=this.d
if(q==null)return null
s=q[this.az(a)]
r=this.aA(s,a)
if(r<0)return null
return s[r].b},
p(a,b,c){var s,r,q,p,o,n,m=this,l=A.c4(m)
l.c.a(b)
l.y[1].a(c)
if(typeof b=="string"){s=m.b
m.aM(s==null?m.b=m.ak():s,b,c)}else if(typeof b=="number"&&(b&0x3fffffff)===b){r=m.c
m.aM(r==null?m.c=m.ak():r,b,c)}else{q=m.d
if(q==null)q=m.d=m.ak()
p=m.az(b)
o=q[p]
if(o==null)q[p]=[m.al(b,c)]
else{n=m.aA(o,b)
if(n>=0)o[n].b=c
else o.push(m.al(b,c))}}},
aH(a,b){if(typeof b=="number"&&(b&0x3fffffff)===b)return this.bQ(this.c,b)
else return this.ca(b)},
ca(a){var s,r,q,p,o=this,n=o.d
if(n==null)return null
s=o.az(a)
r=n[s]
q=o.aA(r,a)
if(q<0)return null
p=r.splice(q,1)[0]
o.b_(p)
if(r.length===0)delete n[s]
return p.b},
M(a){var s=this
if(s.a>0){s.b=s.c=s.d=s.e=s.f=null
s.a=0
s.aj()}},
T(a,b){var s,r,q=this
A.c4(q).i("~(1,2)").a(b)
s=q.e
r=q.r
while(s!=null){b.$2(s.a,s.b)
if(r!==q.r)throw A.i(A.cj(q))
s=s.c}},
aM(a,b,c){var s,r=A.c4(this)
r.c.a(b)
r.y[1].a(c)
s=a[b]
if(s==null)a[b]=this.al(b,c)
else s.b=c},
bQ(a,b){var s
if(a==null)return null
s=a[b]
if(s==null)return null
this.b_(s)
delete a[b]
return s.b},
aj(){this.r=this.r+1&1073741823},
al(a,b){var s=this,r=A.c4(s),q=new A.dl(r.c.a(a),r.y[1].a(b))
if(s.e==null)s.e=s.f=q
else{r=s.f
r.toString
q.d=r
s.f=r.c=q}++s.a
s.aj()
return q},
b_(a){var s=this,r=a.d,q=a.c
if(r==null)s.e=q
else r.c=q
if(q==null)s.f=r
else q.d=r;--s.a
s.aj()},
az(a){return J.T(a)&1073741823},
aA(a,b){var s,r
if(a==null)return-1
s=a.length
for(r=0;r<s;++r)if(J.df(a[r].a,b))return r
return-1},
h(a){return A.dn(this)},
ak(){var s=Object.create(null)
s["<non-identifier-key>"]=s
delete s["<non-identifier-key>"]
return s}}
A.dl.prototype={}
A.f_.prototype={
$1(a){return this.a(a)},
$S:39}
A.f0.prototype={
$2(a,b){return this.a(a,b)},
$S:33}
A.f1.prototype={
$1(a){return this.a(A.Q(a))},
$S:38}
A.a5.prototype={
h(a){return this.aZ(!1)},
aZ(a){var s,r,q,p,o,n=this.bI(),m=this.ai(),l=(a?"Record ":"")+"("
for(s=n.length,r="",q=0;q<s;++q,r=", "){l+=r
p=n[q]
if(typeof p=="string")l=l+p+": "
if(!(q<m.length))return A.f(m,q)
o=m[q]
l=a?l+A.h6(o):l+A.r(o)}l+=")"
return l.charCodeAt(0)==0?l:l},
bI(){var s,r=this.$s
while($.eF.length<=r)B.c.m($.eF,null)
s=$.eF[r]
if(s==null){s=this.bF()
B.c.p($.eF,r,s)}return s},
bF(){var s,r,q,p=this.$r,o=p.indexOf("("),n=p.substring(1,o),m=p.substring(o),l=m==="()"?0:m.replace(/[^,]/g,"").length+1,k=A.h(new Array(l),t.f)
for(s=0;s<l;++s)k[s]=s
if(n!==""){r=n.split(",")
s=r.length
for(q=l;s>0;){--q;--s
B.c.p(k,q,r[s])}}k=A.iK(k,!1,t.K)
k.$flags=3
return k}}
A.b_.prototype={
ai(){return[this.a,this.b]},
B(a,b){if(b==null)return!1
return b instanceof A.b_&&this.$s===b.$s&&J.df(this.a,b.a)&&J.df(this.b,b.b)},
gn(a){return A.aS(this.$s,this.a,this.b,B.h)}}
A.aC.prototype={
ai(){return this.a},
B(a,b){if(b==null)return!1
return b instanceof A.aC&&this.$s===b.$s&&A.jf(this.a,b.a)},
gn(a){return A.aS(this.$s,A.h2(this.a),B.h,B.h)}}
A.ep.prototype={
q(){var s=this.b
if(s===this)throw A.i(new A.aw("Local '"+this.a+"' has not been initialized."))
return s}}
A.aR.prototype={
gt(a){return B.a7},
$io:1}
A.aQ.prototype={$iaQ:1}
A.bs.prototype={
bK(a,b,c,d){var s=A.aa(b,0,c,d,null)
throw A.i(s)},
aP(a,b,c,d){if(b>>>0!==b||b>c)this.bK(a,b,c,d)}}
A.cv.prototype={
gt(a){return B.a8},
$io:1}
A.I.prototype={
gA(a){return a.length},
$iO:1}
A.bq.prototype={$iw:1,$ik:1}
A.br.prototype={
bx(a,b,c,d,e){var s,r,q
t.U.a(d)
a.$flags&2&&A.ar(a,5)
s=a.length
this.aP(a,b,s,"start")
this.aP(a,c,s,"end")
if(b>c)A.A(A.aa(b,0,c,null,null))
r=c-b
if(e<0)A.A(A.Y(e,null))
if(16-e<r)A.A(A.C("Not enough elements"))
q=e!==0||16!==r?d.subarray(e,e+r):d
a.set(q,b)
return},
$iw:1,
$ik:1}
A.cw.prototype={
gt(a){return B.a9},
$io:1,
$ifq:1}
A.cx.prototype={
gt(a){return B.aa},
$io:1}
A.cy.prototype={
gt(a){return B.ab},
$io:1}
A.cz.prototype={
gt(a){return B.ac},
$io:1}
A.cA.prototype={
gt(a){return B.ad},
$io:1}
A.cB.prototype={
gt(a){return B.af},
$io:1}
A.cC.prototype={
gt(a){return B.ag},
$io:1}
A.bt.prototype={
gt(a){return B.ah},
gA(a){return a.length},
$io:1}
A.bu.prototype={
gt(a){return B.ai},
gA(a){return a.length},
$io:1,
$ifu:1}
A.bM.prototype={}
A.bN.prototype={}
A.bO.prototype={}
A.bP.prototype={}
A.a1.prototype={
i(a){return A.bY(v.typeUniverse,this,a)},
F(a){return A.ht(v.typeUniverse,this,a)}}
A.d0.prototype={}
A.eL.prototype={
h(a){return A.R(this.a,null)}}
A.d_.prototype={
h(a){return this.a}}
A.bU.prototype={$iad:1}
A.em.prototype={
$1(a){var s=this.a,r=s.a
s.a=null
r.$0()},
$S:8}
A.el.prototype={
$1(a){var s,r
this.a.a=t.M.a(a)
s=this.b
r=this.c
s.firstChild?s.removeChild(r):s.appendChild(r)},
$S:42}
A.en.prototype={
$0(){this.a.$0()},
$S:7}
A.eo.prototype={
$0(){this.a.$0()},
$S:7}
A.eJ.prototype={
bB(a,b){if(self.setTimeout!=null)self.setTimeout(A.c7(new A.eK(this,b),0),a)
else throw A.i(A.bF("`setTimeout()` not found."))}}
A.eK.prototype={
$0(){this.b.$0()},
$S:0}
A.cV.prototype={
ar(a){var s,r=this,q=r.$ti
q.i("1/?").a(a)
if(a==null)a=q.c.a(a)
if(!r.b)r.a.aN(a)
else{s=r.a
if(q.i("at<1>").b(a))s.aO(a)
else s.aQ(a)}},
au(a,b){var s=this.a
if(this.b)s.ad(new A.U(a,b))
else s.ac(new A.U(a,b))}}
A.eR.prototype={
$1(a){return this.a.$2(0,a)},
$S:5}
A.eS.prototype={
$2(a,b){this.a.$2(1,new A.bh(a,t.l.a(b)))},
$S:32}
A.eX.prototype={
$2(a,b){this.a(A.j(a),b)},
$S:28}
A.U.prototype={
h(a){return A.r(this.a)},
$iu:1,
gX(){return this.b}}
A.cX.prototype={
au(a,b){var s=this.a
if((s.a&30)!==0)throw A.i(A.C("Future already completed"))
s.ac(A.k2(a,b))},
b4(a){return this.au(a,null)}}
A.bJ.prototype={
ar(a){var s,r=this.$ti
r.i("1/?").a(a)
s=this.a
if((s.a&30)!==0)throw A.i(A.C("Future already completed"))
s.aN(r.i("1/").a(a))}}
A.az.prototype={
cc(a){if((this.c&15)!==6)return!0
return this.b.b.aI(t.bG.a(this.d),a.a,t.y,t.K)},
c7(a){var s,r=this,q=r.e,p=null,o=t.z,n=t.K,m=a.a,l=r.b.b
if(t.Q.b(q))p=l.cm(q,m,a.b,o,n,t.l)
else p=l.aI(t.v.a(q),m,o,n)
try{o=r.$ti.i("2/").a(p)
return o}catch(s){if(t.b7.b(A.aJ(s))){if((r.c&1)!==0)throw A.i(A.Y("The error handler of Future.then must return a value of the returned future's type","onError"))
throw A.i(A.Y("The error handler of Future.catchError must return a value of the future's type","onError"))}else throw s}}}
A.E.prototype={
bs(a,b,c){var s,r,q=this.$ti
q.F(c).i("1/(2)").a(a)
s=$.y
if(s===B.i){if(!t.Q.b(b)&&!t.v.b(b))throw A.i(A.fQ(b,"onError",u.c))}else{c.i("@<0/>").F(q.c).i("1(2)").a(a)
b=A.kg(b,s)}r=new A.E(s,c.i("E<0>"))
this.ab(new A.az(r,3,a,b,q.i("@<1>").F(c).i("az<1,2>")))
return r},
aY(a,b,c){var s,r=this.$ti
r.F(c).i("1/(2)").a(a)
s=new A.E($.y,c.i("E<0>"))
this.ab(new A.az(s,19,a,b,r.i("@<1>").F(c).i("az<1,2>")))
return s},
bS(a){this.a=this.a&1|16
this.c=a},
a2(a){this.a=a.a&30|this.a&1
this.c=a.c},
ab(a){var s,r=this,q=r.a
if(q<=3){a.a=t.d.a(r.c)
r.c=a}else{if((q&4)!==0){s=t.c.a(r.c)
if((s.a&24)===0){s.ab(a)
return}r.a2(s)}A.da(null,null,r.b,t.M.a(new A.et(r,a)))}},
aU(a){var s,r,q,p,o,n,m=this,l={}
l.a=a
if(a==null)return
s=m.a
if(s<=3){r=t.d.a(m.c)
m.c=a
if(r!=null){q=a.a
for(p=a;q!=null;p=q,q=o)o=q.a
p.a=r}}else{if((s&4)!==0){n=t.c.a(m.c)
if((n.a&24)===0){n.aU(a)
return}m.a2(n)}l.a=m.a4(a)
A.da(null,null,m.b,t.M.a(new A.ex(l,m)))}},
Y(){var s=t.d.a(this.c)
this.c=null
return this.a4(s)},
a4(a){var s,r,q
for(s=a,r=null;s!=null;r=s,s=q){q=s.a
s.a=r}return r},
aQ(a){var s,r=this
r.$ti.c.a(a)
s=r.Y()
r.a=8
r.c=a
A.aA(r,s)},
bE(a){var s,r,q=this
if((a.a&16)!==0){s=q.b===a.b
s=!(s||s)}else s=!1
if(s)return
r=q.Y()
q.a2(a)
A.aA(q,r)},
ad(a){var s=this.Y()
this.bS(a)
A.aA(this,s)},
aN(a){var s=this.$ti
s.i("1/").a(a)
if(s.i("at<1>").b(a)){this.aO(a)
return}this.bD(a)},
bD(a){var s=this
s.$ti.c.a(a)
s.a^=2
A.da(null,null,s.b,t.M.a(new A.ev(s,a)))},
aO(a){A.fv(this.$ti.i("at<1>").a(a),this,!1)
return},
ac(a){this.a^=2
A.da(null,null,this.b,t.M.a(new A.eu(this,a)))},
$iat:1}
A.et.prototype={
$0(){A.aA(this.a,this.b)},
$S:0}
A.ex.prototype={
$0(){A.aA(this.b,this.a.a)},
$S:0}
A.ew.prototype={
$0(){A.fv(this.a.a,this.b,!0)},
$S:0}
A.ev.prototype={
$0(){this.a.aQ(this.b)},
$S:0}
A.eu.prototype={
$0(){this.a.ad(this.b)},
$S:0}
A.eA.prototype={
$0(){var s,r,q,p,o,n,m,l,k=this,j=null
try{q=k.a.a
j=q.b.b.cl(t.O.a(q.d),t.z)}catch(p){s=A.aJ(p)
r=A.aH(p)
if(k.c&&t.n.a(k.b.a.c).a===s){q=k.a
q.c=t.n.a(k.b.a.c)}else{q=s
o=r
if(o==null)o=A.fo(q)
n=k.a
n.c=new A.U(q,o)
q=n}q.b=!0
return}if(j instanceof A.E&&(j.a&24)!==0){if((j.a&16)!==0){q=k.a
q.c=t.n.a(j.c)
q.b=!0}return}if(j instanceof A.E){m=k.b.a
l=new A.E(m.b,m.$ti)
j.bs(new A.eB(l,m),new A.eC(l),t.H)
q=k.a
q.c=l
q.b=!1}},
$S:0}
A.eB.prototype={
$1(a){this.a.bE(this.b)},
$S:8}
A.eC.prototype={
$2(a,b){A.b3(a)
t.l.a(b)
this.a.ad(new A.U(a,b))},
$S:20}
A.ez.prototype={
$0(){var s,r,q,p,o,n,m,l
try{q=this.a
p=q.a
o=p.$ti
n=o.c
m=n.a(this.b)
q.c=p.b.b.aI(o.i("2/(1)").a(p.d),m,o.i("2/"),n)}catch(l){s=A.aJ(l)
r=A.aH(l)
q=s
p=r
if(p==null)p=A.fo(q)
o=this.a
o.c=new A.U(q,p)
o.b=!0}},
$S:0}
A.ey.prototype={
$0(){var s,r,q,p,o,n,m,l=this
try{s=t.n.a(l.a.a.c)
p=l.b
if(p.a.cc(s)&&p.a.e!=null){p.c=p.a.c7(s)
p.b=!1}}catch(o){r=A.aJ(o)
q=A.aH(o)
p=t.n.a(l.a.a.c)
if(p.a===r){n=l.b
n.c=p
p=n}else{p=r
n=q
if(n==null)n=A.fo(p)
m=l.b
m.c=new A.U(p,n)
p=m}p.b=!0}},
$S:0}
A.cW.prototype={}
A.bB.prototype={
gA(a){var s,r,q=this,p={},o=new A.E($.y,t.aQ)
p.a=0
s=q.$ti
r=s.i("~(1)?").a(new A.dA(p,q))
t.bp.a(new A.dB(p,o))
A.hj(q.a,q.b,r,!1,s.c)
return o}}
A.dA.prototype={
$1(a){this.b.$ti.c.a(a);++this.a.a},
$S(){return this.b.$ti.i("~(1)")}}
A.dB.prototype={
$0(){var s=this.b,r=s.$ti,q=r.i("1/").a(this.a.a),p=s.Y()
r.c.a(q)
s.a=8
s.c=q
A.aA(s,p)},
$S:0}
A.d3.prototype={}
A.c1.prototype={$ihh:1}
A.d1.prototype={
cn(a){var s,r,q
t.M.a(a)
try{if(B.i===$.y){a.$0()
return}A.hL(null,null,this,a,t.H)}catch(q){s=A.aJ(q)
r=A.aH(q)
A.eV(A.b3(s),t.l.a(r))}},
co(a,b,c){var s,r,q
c.i("~(0)").a(a)
c.a(b)
try{if(B.i===$.y){a.$1(b)
return}A.hM(null,null,this,a,b,t.H,c)}catch(q){s=A.aJ(q)
r=A.aH(q)
A.eV(A.b3(s),t.l.a(r))}},
bW(a){return new A.eH(this,t.M.a(a))},
bX(a,b){return new A.eI(this,b.i("~(0)").a(a),b)},
cl(a,b){b.i("0()").a(a)
if($.y===B.i)return a.$0()
return A.hL(null,null,this,a,b)},
aI(a,b,c,d){c.i("@<0>").F(d).i("1(2)").a(a)
d.a(b)
if($.y===B.i)return a.$1(b)
return A.hM(null,null,this,a,b,c,d)},
cm(a,b,c,d,e,f){d.i("@<0>").F(e).F(f).i("1(2,3)").a(a)
e.a(b)
f.a(c)
if($.y===B.i)return a.$2(b,c)
return A.kh(null,null,this,a,b,c,d,e,f)},
bq(a,b,c,d){return b.i("@<0>").F(c).F(d).i("1(2,3)").a(a)}}
A.eH.prototype={
$0(){return this.a.cn(this.b)},
$S:0}
A.eI.prototype={
$1(a){var s=this.c
return this.a.co(this.b,s.a(a),s)},
$S(){return this.c.i("~(0)")}}
A.eW.prototype={
$0(){A.iy(this.a,this.b)},
$S:0}
A.x.prototype={
gbj(a){return new A.cu(a,a.length,A.b9(a).i("cu<x.E>"))},
c4(a,b){if(!(b>=0&&b<a.length))return A.f(a,b)
return a[b]},
c5(a,b,c,d){var s,r
A.b9(a).i("x.E?").a(d)
s=a.length
A.cI(b,c,s)
for(r=b;r<c;++r){if(!(r>=0&&r<s))return A.f(a,r)
a[r]=d}},
h(a){return A.h_(a,"[","]")}}
A.bp.prototype={
gA(a){return this.a},
h(a){return A.dn(this)},
$ia9:1}
A.dp.prototype={
$2(a,b){var s,r=this.a
if(!r.a)this.b.a+=", "
r.a=!1
r=this.b
s=A.r(a)
r.a=(r.a+=s)+": "
s=A.r(b)
r.a+=s},
$S:19}
A.bZ.prototype={}
A.aP.prototype={
T(a,b){this.a.T(0,this.$ti.i("~(1,2)").a(b))},
gA(a){return this.a.a},
h(a){return A.dn(this.a)},
$ia9:1}
A.bD.prototype={}
A.b0.prototype={}
A.eP.prototype={
$0(){var s,r
try{s=new TextDecoder("utf-8",{fatal:true})
return s}catch(r){}return null},
$S:13}
A.eO.prototype={
$0(){var s,r
try{s=new TextDecoder("utf-8",{fatal:false})
return s}catch(r){}return null},
$S:13}
A.cc.prototype={
cg(a3,a4,a5){var s,r,q,p,o,n,m,l,k,j,i,h,g,f,e,d,c,b,a,a0="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/",a1="Invalid base64 encoding length ",a2=a3.length
a5=A.cI(a4,a5,a2)
s=$.ii()
for(r=s.length,q=a4,p=q,o=null,n=-1,m=-1,l=0;q<a5;q=k){k=q+1
if(!(q<a2))return A.f(a3,q)
j=a3.charCodeAt(q)
if(j===37){i=k+2
if(i<=a5){if(!(k<a2))return A.f(a3,k)
h=A.eZ(a3.charCodeAt(k))
g=k+1
if(!(g<a2))return A.f(a3,g)
f=A.eZ(a3.charCodeAt(g))
e=h*16+f-(f&256)
if(e===37)e=-1
k=i}else e=-1}else e=j
if(0<=e&&e<=127){if(!(e>=0&&e<r))return A.f(s,e)
d=s[e]
if(d>=0){if(!(d<64))return A.f(a0,d)
e=a0.charCodeAt(d)
if(e===j)continue
j=e}else{if(d===-1){if(n<0){g=o==null?null:o.a.length
if(g==null)g=0
n=g+(q-p)
m=q}++l
if(j===61)continue}j=e}if(d!==-2){if(o==null){o=new A.J("")
g=o}else g=o
g.a+=B.a.l(a3,p,q)
c=A.ax(j)
g.a+=c
p=k
continue}}throw A.i(A.M("Invalid base64 data",a3,q))}if(o!=null){a2=B.a.l(a3,p,a5)
a2=o.a+=a2
r=a2.length
if(n>=0)A.fR(a3,m,a5,n,l,r)
else{b=B.b.G(r-1,4)+1
if(b===1)throw A.i(A.M(a1,a3,a5))
while(b<4){a2+="="
o.a=a2;++b}}a2=o.a
return B.a.U(a3,a4,a5,a2.charCodeAt(0)==0?a2:a2)}a=a5-a4
if(n>=0)A.fR(a3,m,a5,n,l,a)
else{b=B.b.G(a,4)
if(b===1)throw A.i(A.M(a1,a3,a5))
if(b>1)a3=B.a.U(a3,a5,a5,b===2?"==":"=")}return a3}}
A.dg.prototype={}
A.ch.prototype={}
A.ck.prototype={}
A.dH.prototype={
c_(a){return new A.eN(this.a).bH(t.L.a(a),0,null,!0)}}
A.eN.prototype={
bH(a,b,c,d){var s,r,q,p,o,n,m,l=this
t.L.a(a)
s=A.cI(b,c,a.length)
if(b===s)return""
if(a instanceof Uint8Array){r=a
q=r
p=0}else{q=A.jF(a,b,s)
s-=b
p=b
b=0}if(s-b>=15){o=l.a
n=A.jE(o,q,b,s)
if(n!=null){if(!o)return n
if(n.indexOf("\ufffd")<0)return n}}n=l.ae(q,b,s,!0)
o=l.b
if((o&1)!==0){m=A.jG(o)
l.b=0
throw A.i(A.M(m,a,p+l.c))}return n},
ae(a,b,c,d){var s,r,q=this
if(c-b>1000){s=B.b.j(b+c,2)
r=q.ae(a,b,s,!1)
if((q.b&1)!==0)return r
return r+q.ae(a,s,c,d)}return q.c0(a,b,c,d)},
c0(a,b,a0,a1){var s,r,q,p,o,n,m,l,k=this,j="AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAFFFFFFFFFFFFFFFFGGGGGGGGGGGGGGGGHHHHHHHHHHHHHHHHHHHHHHHHHHHIHHHJEEBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBKCCCCCCCCCCCCDCLONNNMEEEEEEEEEEE",i=" \x000:XECCCCCN:lDb \x000:XECCCCCNvlDb \x000:XECCCCCN:lDb AAAAA\x00\x00\x00\x00\x00AAAAA00000AAAAA:::::AAAAAGG000AAAAA00KKKAAAAAG::::AAAAA:IIIIAAAAA000\x800AAAAA\x00\x00\x00\x00 AAAAA",h=65533,g=k.b,f=k.c,e=new A.J(""),d=b+1,c=a.length
if(!(b>=0&&b<c))return A.f(a,b)
s=a[b]
A:for(r=k.a;;){for(;;d=o){if(!(s>=0&&s<256))return A.f(j,s)
q=j.charCodeAt(s)&31
f=g<=32?s&61694>>>q:(s&63|f<<6)>>>0
p=g+q
if(!(p>=0&&p<144))return A.f(i,p)
g=i.charCodeAt(p)
if(g===0){p=A.ax(f)
e.a+=p
if(d===a0)break A
break}else if((g&1)!==0){if(r)switch(g){case 69:case 67:p=A.ax(h)
e.a+=p
break
case 65:p=A.ax(h)
e.a+=p;--d
break
default:p=A.ax(h)
e.a=(e.a+=p)+p
break}else{k.b=g
k.c=d-1
return""}g=0}if(d===a0)break A
o=d+1
if(!(d>=0&&d<c))return A.f(a,d)
s=a[d]}o=d+1
if(!(d>=0&&d<c))return A.f(a,d)
s=a[d]
if(s<128){for(;;){if(!(o<a0)){n=a0
break}m=o+1
if(!(o>=0&&o<c))return A.f(a,o)
s=a[o]
if(s>=128){n=m-1
o=m
break}o=m}if(n-d<20)for(l=d;l<n;++l){if(!(l<c))return A.f(a,l)
p=A.ax(a[l])
e.a+=p}else{p=A.ha(a,d,n)
e.a+=p}if(n===a0)break A
d=o}else d=o}if(a1&&g>32)if(r){c=A.ax(h)
e.a+=c}else{k.b=77
k.c=a0
return""}k.b=g
k.c=f
c=e.a
return c.charCodeAt(0)==0?c:c}}
A.ds.prototype={
$2(a,b){var s,r,q
t.cm.a(a)
s=this.b
r=this.a
q=(s.a+=r.a)+a.a
s.a=q
s.a=q+": "
q=A.aN(b)
s.a+=q
r.a=", "},
$S:14}
A.eq.prototype={
h(a){return this.a3()}}
A.u.prototype={
gX(){return A.iP(this)}}
A.ca.prototype={
h(a){var s=this.a
if(s!=null)return"Assertion failed: "+A.aN(s)
return"Assertion failed"}}
A.ad.prototype={}
A.a3.prototype={
gag(){return"Invalid argument"+(!this.a?"(s)":"")},
gaf(){return""},
h(a){var s=this,r=s.c,q=r==null?"":" ("+r+")",p=s.d,o=p==null?"":": "+A.r(p),n=s.gag()+q+o
if(!s.a)return n
return n+s.gaf()+": "+A.aN(s.gaB())},
gaB(){return this.b}}
A.bx.prototype={
gaB(){return A.hE(this.b)},
gag(){return"RangeError"},
gaf(){var s,r=this.e,q=this.f
if(r==null)s=q!=null?": Not less than or equal to "+A.r(q):""
else if(q==null)s=": Not greater than or equal to "+A.r(r)
else if(q>r)s=": Not in inclusive range "+A.r(r)+".."+A.r(q)
else s=q<r?": Valid value range is empty":": Only valid value is "+A.r(r)
return s}}
A.cn.prototype={
gaB(){return A.j(this.b)},
gag(){return"RangeError"},
gaf(){if(A.j(this.b)<0)return": index must not be negative"
var s=this.f
if(s===0)return": no indices are valid"
return": index should be less than "+s},
gA(a){return this.f}}
A.cD.prototype={
h(a){var s,r,q,p,o,n,m,l,k=this,j={},i=new A.J("")
j.a=""
s=k.c
for(r=s.length,q=0,p="",o="";q<r;++q,o=", "){n=s[q]
i.a=p+o
p=A.aN(n)
p=i.a+=p
j.a=", "}k.d.T(0,new A.ds(j,i))
m=A.aN(k.a)
l=i.h(0)
return"NoSuchMethodError: method not found: '"+k.b.a+"'\nReceiver: "+m+"\nArguments: ["+l+"]"}}
A.bE.prototype={
h(a){return"Unsupported operation: "+this.a}}
A.cN.prototype={
h(a){return"UnimplementedError: "+this.a}}
A.aX.prototype={
h(a){return"Bad state: "+this.a}}
A.ci.prototype={
h(a){var s=this.a
if(s==null)return"Concurrent modification during iteration."
return"Concurrent modification during iteration: "+A.aN(s)+"."}}
A.cE.prototype={
h(a){return"Out of Memory"},
gX(){return null},
$iu:1}
A.bA.prototype={
h(a){return"Stack Overflow"},
gX(){return null},
$iu:1}
A.es.prototype={
h(a){return"Exception: "+this.a}}
A.a8.prototype={
h(a){var s,r,q,p,o,n,m,l,k,j,i,h=this.a,g=""!==h?"FormatException: "+h:"FormatException",f=this.c,e=this.b
if(typeof e=="string"){if(f!=null)s=f<0||f>e.length
else s=!1
if(s)f=null
if(f==null){if(e.length>78)e=B.a.l(e,0,75)+"..."
return g+"\n"+e}for(r=e.length,q=1,p=0,o=!1,n=0;n<f;++n){if(!(n<r))return A.f(e,n)
m=e.charCodeAt(n)
if(m===10){if(p!==n||!o)++q
p=n+1
o=!1}else if(m===13){++q
p=n+1
o=!0}}g=q>1?g+(" (at line "+q+", character "+(f-p+1)+")\n"):g+(" (at character "+(f+1)+")\n")
for(n=f;n<r;++n){if(!(n>=0))return A.f(e,n)
m=e.charCodeAt(n)
if(m===10||m===13){r=n
break}}l=""
if(r-p>78){k="..."
if(f-p<75){j=p+75
i=p}else{if(r-f<75){i=r-75
j=r
k=""}else{i=f-36
j=f+36}l="..."}}else{j=r
i=p
k=""}return g+l+B.a.l(e,i,j)+k+"\n"+B.a.bw(" ",f-i+l.length)+"^\n"}else return f!=null?g+(" (at offset "+A.r(f)+")"):g}}
A.v.prototype={
gn(a){return A.p.prototype.gn.call(this,0)},
h(a){return"null"}}
A.p.prototype={$ip:1,
B(a,b){return this===b},
gn(a){return A.cG(this)},
h(a){return"Instance of '"+A.cH(this)+"'"},
bl(a,b){throw A.i(A.h1(this,t.A.a(b)))},
gt(a){return A.kz(this)},
toString(){return this.h(this)}}
A.d4.prototype={
h(a){return""},
$ian:1}
A.J.prototype={
gA(a){return this.a.length},
h(a){var s=this.a
return s.charCodeAt(0)==0?s:s},
$iiW:1}
A.dG.prototype={
$2(a,b){throw A.i(A.M("Illegal IPv6 address, "+a,this.a,b))},
$S:15}
A.c_.prototype={
gaX(){var s,r,q,p,o=this,n=o.w
if(n===$){s=o.a
r=s.length!==0?s+":":""
q=o.c
p=q==null
if(!p||s==="file"){s=r+"//"
r=o.b
if(r.length!==0)s=s+r+"@"
if(!p)s+=q
r=o.d
if(r!=null)s=s+":"+A.r(r)}else s=r
s+=o.e
r=o.f
if(r!=null)s=s+"?"+r
r=o.r
if(r!=null)s=s+"#"+r
n=o.w=s.charCodeAt(0)==0?s:s}return n},
gn(a){var s,r=this,q=r.y
if(q===$){s=B.a.gn(r.gaX())
r.y!==$&&A.i4("hashCode")
r.y=s
q=s}return q},
gbu(){return this.b},
gaw(){var s=this.c
if(s==null)return""
if(B.a.E(s,"[")&&!B.a.u(s,"v",1))return B.a.l(s,1,s.length-1)
return s},
gaF(){var s=this.d
return s==null?A.hu(this.a):s},
gbp(){var s=this.f
return s==null?"":s},
gav(){var s=this.r
return s==null?"":s},
gbg(){return this.c!=null},
gbi(){return this.f!=null},
gbh(){return this.r!=null},
h(a){return this.gaX()},
B(a,b){var s,r,q,p=this
if(b==null)return!1
if(p===b)return!0
s=!1
if(t.R.b(b))if(p.a===b.gaK())if(p.c!=null===b.gbg())if(p.b===b.gbu())if(p.gaw()===b.gaw())if(p.gaF()===b.gaF())if(p.e===b.gbo()){r=p.f
q=r==null
if(!q===b.gbi()){if(q)r=""
if(r===b.gbp()){r=p.r
q=r==null
if(!q===b.gbh()){s=q?"":r
s=s===b.gav()}}}}return s},
$ibG:1,
gaK(){return this.a},
gbo(){return this.e}}
A.dF.prototype={
gbt(){var s,r,q,p,o=this,n=null,m=o.c
if(m==null){m=o.b
if(0>=m.length)return A.f(m,0)
s=o.a
m=m[0]+1
r=B.a.a6(s,"?",m)
q=s.length
if(r>=0){p=A.c0(s,r+1,q,256,!1,!1)
q=r}else p=n
m=o.c=new A.cY("data","",n,n,A.c0(s,m,q,128,!1,!1),p,n)}return m},
h(a){var s,r=this.b
if(0>=r.length)return A.f(r,0)
s=this.a
return r[0]===-1?"data:"+s:s}}
A.d2.prototype={
gbg(){return this.c>0},
gbi(){return this.f<this.r},
gbh(){return this.r<this.a.length},
gaK(){var s=this.w
return s==null?this.w=this.bG():s},
bG(){var s,r=this,q=r.b
if(q<=0)return""
s=q===4
if(s&&B.a.E(r.a,"http"))return"http"
if(q===5&&B.a.E(r.a,"https"))return"https"
if(s&&B.a.E(r.a,"file"))return"file"
if(q===7&&B.a.E(r.a,"package"))return"package"
return B.a.l(r.a,0,q)},
gbu(){var s=this.c,r=this.b+3
return s>r?B.a.l(this.a,r,s-1):""},
gaw(){var s=this.c
return s>0?B.a.l(this.a,s,this.d):""},
gaF(){var s,r=this
if(r.c>0&&r.d+1<r.e)return A.kG(B.a.l(r.a,r.d+1,r.e))
s=r.b
if(s===4&&B.a.E(r.a,"http"))return 80
if(s===5&&B.a.E(r.a,"https"))return 443
return 0},
gbo(){return B.a.l(this.a,this.e,this.f)},
gbp(){var s=this.f,r=this.r
return s<r?B.a.l(this.a,s+1,r):""},
gav(){var s=this.r,r=this.a
return s<r.length?B.a.aL(r,s+1):""},
gn(a){var s=this.x
return s==null?this.x=B.a.gn(this.a):s},
B(a,b){if(b==null)return!1
if(this===b)return!0
return t.R.b(b)&&this.a===b.h(0)},
h(a){return this.a},
$ibG:1}
A.cY.prototype={}
A.dt.prototype={
h(a){return"Promise was rejected with a value of `"+(this.a?"undefined":"null")+"`."}}
A.f7.prototype={
$1(a){return this.a.ar(this.b.i("0/?").a(a))},
$S:5}
A.f8.prototype={
$1(a){if(a==null)return this.a.b4(new A.dt(a===undefined))
return this.a.b4(a)},
$S:5}
A.eD.prototype={
K(){return Math.random()},
cf(){return Math.random()<0.5}}
A.bQ.prototype={
aa(a){var s,r,q,p,o,n,m,l=this,k=4294967296
do{s=a>>>0
a=B.b.j(a-s,k)
r=a>>>0
a=B.b.j(a-r,k)
q=(~s>>>0)+(s<<21>>>0)
p=q>>>0
r=(~r>>>0)+((r<<21|s>>>11)>>>0)+B.b.j(q-p,k)>>>0
q=((p^(p>>>24|r<<8))>>>0)*265
s=q>>>0
r=((r^r>>>24)>>>0)*265+B.b.j(q-s,k)>>>0
q=((s^(s>>>14|r<<18))>>>0)*21
s=q>>>0
r=((r^r>>>14)>>>0)*21+B.b.j(q-s,k)>>>0
s=(s^(s>>>28|r<<4))>>>0
r=(r^r>>>28)>>>0
q=(s<<31>>>0)+s
p=q>>>0
o=B.b.j(q-p,k)
q=l.a*1037
n=l.a=q>>>0
m=l.b*1037+B.b.j(q-n,k)>>>0
l.b=m
n=(n^p)>>>0
l.a=n
o=(m^r+((r<<31|s>>>1)>>>0)+o>>>0)>>>0
l.b=o}while(a!==0)
if(o===0&&n===0)l.a=23063
l.P()
l.P()
l.P()
l.P()},
P(){var s=this,r=s.a,q=4294901760*r,p=q>>>0,o=55905*r,n=o>>>0,m=n+p+s.b
r=m>>>0
s.a=r
s.b=B.b.j(o-n+(q-p)+(m-r),4294967296)>>>0},
K(){var s,r=this
r.P()
s=r.a
r.P()
return((s&67108863)*134217728+(r.a&134217727))/9007199254740992}}
A.V.prototype={
ga7(){var s,r,q,p=$.d
if(p==null)p=$.d=A.m()
s=p.a
r=s.x
r===$&&A.l("_scratch")
s.$2("f2d_body_get_position",t.r.a(A.h([this.b,this.c,r],t.a)))
q=p.bP()
p=new A.e(new Float32Array(2))
p.k(q.a,q.b)
return p},
C(a4,a5){var s,r,q,p,o,n,m,l,k,j,i,h,g,f,e,d,c,b,a,a0=this,a1=null,a2="_scratch",a3=a0.a
if(a3.r)A.A(A.C("Cannot create a shape while the world is stepping. Collision callbacks run inside step(); create after step() returns instead. Destroy operations are deferred automatically."))
s=a5.a
r=new A.bR([1,s.f,a5.b,!1,!1,!1,!1,s.a,0,!0,!1,-1,s.b,0,0,!0,0])
A:{if(a4 instanceof A.W){s=$.d
if(s==null)s=$.d=A.m()
q=a4.a.a
p=q[0]
q=q[1]
s.ap(r)
o=s.a
n=o.x
n===$&&A.l(a2)
o.$2("f2d_create_circle_shape",t.r.a(A.h([a0.b,a0.c,n+128,p,q,a4.b,n],t.a)))
s=s.L()
break A}m=a4 instanceof A.a_
l=a1
if(m){l=a4.a
s=l
s=s!=null}else s=!1
if(s){k=m?l:a4.a
if(k==null)k=t.Y.a(k)
j=a4.b
s=$.d
if(s==null)s=$.d=A.m()
q=t.u
p=A.h([],q)
for(o=k.length,i=0;i<k.length;k.length===o||(0,A.S)(k),++i){n=k[i].a
B.c.S(p,A.h([n[0],n[1]],q))}t.o.a(p)
s.ap(r)
q=s.a
h=q.b2(p.length*4)
q.aJ(h,p)
o=q.x
o===$&&A.l(a2)
if(q.$2("f2d_create_polygon_shape",t.r.a(A.h([a0.b,a0.c,o+128,h,p.length/2|0,j,o],t.a)))===0)A.A(A.Y("The points do not form a valid convex hull: at least three distinct, non-collinear points are required",a1))
s=s.L()
break A}g=a1
f=a1
s=!1
if(m){e=a4.c
d=e!=null
if(d){f=e==null?A.c(e):e
g=a4.d
s=g!=null}}else d=!1
if(s){c=d?g:a4.d
if(c==null)c=A.c(c)
j=a4.b
s=$.d
if(s==null)s=$.d=A.m()
A.c(f)
s.ap(r)
q=s.a
p=q.x
p===$&&A.l(a2)
q.$2("f2d_create_box_shape",t.r.a(A.h([a0.b,a0.c,p+128,f,c,0,0,1,0,j,p],t.a)))
s=s.L()
break A}s=m?A.A(A.C("Unreachable polygon configuration")):a1}b=s.a
a=s.b
s=a5.d
if(s!=null)a3.c.p(0,new A.N(b,a),s)
return new A.aW(a3,b,a)},
b6(a){var s,r,q,p,o,n,m,l,k,j,i,h,g,f,e,d,c,b=this.a
if(b.r)A.A(A.C("Cannot create a chain while the world is stepping. Collision callbacks run inside step(); create after step() returns instead. Destroy operations are deferred automatically."))
s=a.a
if(s.length<4)throw A.i(A.Y("A chain needs at least four points",null))
r=$.d
if(r==null)r=$.d=A.m()
q=this.b
p=this.c
o=t.u
n=A.h([],o)
for(m=s.length,l=0;l<s.length;s.length===m||(0,A.S)(s),++l){k=s[l].a
B.c.S(n,A.h([k[0],k[1]],o))}s=A.h([],o)
for(m=a.b,l=0;l<1;++l){j=m[l]
B.c.S(s,A.h([j.a,j.b,0,0,0,j.f],o))}o=t.o
o.a(s)
o.a(n)
i=n.length*4
o=r.a
h=o.b2(i+s.length*4)
o.aJ(h,n)
m=h+i
o.aJ(m,s)
g=A.by(1)
f=A.by(-1)
n=n.length
s=s.length
k=a.e?1:0
e=o.x
e===$&&A.l("_scratch")
o.$2("f2d_create_chain",t.r.a(A.h([q,p,h,n/2|0,m,s/6|0,g.a,g.b,f.a,f.b,0,k,0,e],t.a)))
g=r.L()
d=g.a
c=g.b
b.f.p(0,new A.N(d,c),new A.N(q,p))
return new A.ce(b,d,c)},
B(a,b){if(b==null)return!1
return b instanceof A.V&&b.a===this.a&&b.b===this.b&&b.c===this.c},
gn(a){return A.aS(this.a,this.b,this.c,B.h)},
h(a){return"Body("+this.b+")"}}
A.ce.prototype={
B(a,b){if(b==null)return!1
return b instanceof A.ce&&b.a===this.a&&b.b===this.b&&b.c===this.c},
gn(a){return A.aS(this.a,this.b,this.c,B.h)},
h(a){return"Chain("+this.b+")"}}
A.cl.prototype={}
A.cm.prototype={}
A.H.prototype={}
A.e2.prototype={}
A.dh.prototype={}
A.dz.prototype={}
A.di.prototype={}
A.aK.prototype={
a3(){return"BodyType."+this.b}}
A.am.prototype={
a3(){return"ShapeType."+this.b}}
A.Z.prototype={
a3(){return"JointType."+this.b}}
A.ay.prototype={}
A.W.prototype={}
A.bd.prototype={}
A.aV.prototype={}
A.a_.prototype={}
A.aM.prototype={
gA(a){return this.r}}
A.dj.prototype={
gA(a){var s=$.d
if(s==null)s=$.d=A.m()
return s.a.$2("f2d_distance_joint_get_length",t.r.a(A.h([this.b,this.c],t.a)))}}
A.au.prototype={}
A.bo.prototype={
b9(){var s,r,q,p,o,n=this,m=n.a
if(m.r){B.c.m(m.w,n.gc1())
return}s=$.d
if(s==null)s=$.d=A.m()
r=n.b
q=n.c
p=t.a
o=t.r
if(s.a.$2("f2d_joint_is_valid",o.a(A.h([r,q],p)))===0)return
m.e.aH(0,new A.N(r,q))
m=$.d
if(m==null)m=$.d=A.m()
m.a.$2("f2d_destroy_joint",o.a(A.h([r,q],p)))},
B(a,b){if(b==null)return!1
return b instanceof A.bo&&b.a===this.a&&b.b===this.b&&b.c===this.c},
gn(a){return A.aS(this.a,this.b,this.c,B.h)},
h(a){var s,r=$.d
if(r==null)r=$.d=A.m()
s=this.b
r=B.f.W(r.a.$2("f2d_joint_get_type",t.r.a(A.h([s,this.c],t.a))))
if(!(r>=0&&r<8))return A.f(B.t,r)
return B.t[r].b+" Joint("+s+")"}}
A.dr.prototype={}
A.dq.prototype={}
A.cJ.prototype={}
A.dx.prototype={}
A.e1.prototype={}
A.cT.prototype={}
A.al.prototype={
V(a){var s,r=this.a,q=a.a,p=q[0],o=this.b
q=q[1]
s=new A.e(new Float32Array(2))
s.k(r*p-o*q,o*p+r*q)
return s},
B(a,b){if(b==null)return!1
return b instanceof A.al&&b.a===this.a&&b.b===this.b},
gn(a){return A.aS(this.a,this.b,B.h,B.h)},
h(a){return"Rot(cos: "+A.r(this.a)+", sin: "+A.r(this.b)+")"}}
A.aZ.prototype={
bV(a){var s,r=this.b,q=r.a,p=a.a,o=p[0]
r=r.b
p=p[1]
s=new A.e(new Float32Array(2))
s.k(q*o-r*p,r*o+q*p)
s.m(0,this.a)
return s},
h(a){return"Transform(position: "+this.a.h(0)+", rotation: "+this.b.h(0)+")"}}
A.c8.prototype={
h(a){return"Aabb("+this.a.h(0)+", "+this.b.h(0)+")"}}
A.aW.prototype={
gb1(){var s,r,q,p=$.d
if(p==null)p=$.d=A.m()
s=p.a
r=s.x
r===$&&A.l("_scratch")
s.$2("f2d_shape_get_body",t.r.a(A.h([this.b,this.c,r],t.a)))
q=p.L()
return new A.V(this.a,q.a,q.b)},
gbv(){var s,r,q,p,o,n,m,l,k,j,i,h,g,f,e,d,c,b="_scratch",a="_f32",a0=$.d
if(a0==null)a0=$.d=A.m()
s=this.b
r=this.c
q=t.a
p=t.r
a0=B.f.W(a0.a.$2("f2d_shape_get_type",p.a(A.h([s,r],q))))
if(!(a0>=0&&a0<5))return A.f(B.u,a0)
switch(B.u[a0].a){case 0:a0=$.d
a0=(a0==null?$.d=A.m():a0).a
o=a0.x
o===$&&A.l(b)
a0.$2("f2d_shape_get_circle",p.a(A.h([s,r,o],q)))
a0=a0.d
a0===$&&A.l(a)
n=A.c(a0[B.b.j(o,4)])
m=A.c(a0[B.b.j(o+4,4)])
l=A.c(a0[B.b.j(o+8,4)])
a0=new A.e(new Float32Array(2))
a0.k(n,m)
return new A.W(a0,l)
case 1:a0=$.d
a0=(a0==null?$.d=A.m():a0).a
o=a0.x
o===$&&A.l(b)
a0.$2("f2d_shape_get_capsule",p.a(A.h([s,r,o],q)))
a0=a0.d
a0===$&&A.l(a)
o=[A.c(a0[B.b.j(o,4)]),A.c(a0[B.b.j(o+4,4)]),A.c(a0[B.b.j(o+8,4)]),A.c(a0[B.b.j(o+12,4)]),A.c(a0[B.b.j(o+16,4)])]
k=o[0]
j=o[1]
i=o[2]
h=o[3]
l=o[4]
a0=new A.e(new Float32Array(2))
a0.k(k,j)
s=new A.e(new Float32Array(2))
s.k(i,h)
return new A.bd(a0,s,l)
case 2:a0=$.d
a0=(a0==null?$.d=A.m():a0).a
o=a0.x
o===$&&A.l(b)
a0.$2("f2d_shape_get_segment",p.a(A.h([s,r,o],q)))
a0=a0.d
a0===$&&A.l(a)
o=[A.c(a0[B.b.j(o,4)]),A.c(a0[B.b.j(o+4,4)]),A.c(a0[B.b.j(o+8,4)]),A.c(a0[B.b.j(o+12,4)])]
g=o[0]
f=o[1]
e=o[2]
d=o[3]
a0=new A.e(new Float32Array(2))
a0.k(g,f)
s=new A.e(new Float32Array(2))
s.k(e,d)
return new A.aV(a0,s)
case 4:a0=$.d
a0=(a0==null?$.d=A.m():a0).a
o=a0.x
o===$&&A.l(b)
a0.$2("f2d_shape_get_chain_segment",p.a(A.h([s,r,o],q)))
a0=a0.d
a0===$&&A.l(a)
o=[A.c(a0[B.b.j(o,4)]),A.c(a0[B.b.j(o+4,4)]),A.c(a0[B.b.j(o+8,4)]),A.c(a0[B.b.j(o+12,4)])]
g=o[0]
f=o[1]
e=o[2]
d=o[3]
a0=new A.e(new Float32Array(2))
a0.k(g,f)
s=new A.e(new Float32Array(2))
s.k(e,d)
return new A.aV(a0,s)
case 3:a0=$.d
a0=(a0==null?$.d=A.m():a0).a
o=a0.x
o===$&&A.l(b)
q=a0.aG(o+4,B.f.W(a0.$2("f2d_shape_get_polygon",p.a(A.h([s,r,o],q))))*2)
a0=a0.d
a0===$&&A.l(a)
o=A.c(a0[B.b.j(o,4)])
a0=A.h([],t.F)
for(c=0;s=q.length,c<s;c+=2){r=q[c]
p=c+1
if(!(p<s))return A.f(q,p)
p=q[p]
s=new Float32Array(2)
s[1]=p
s[0]=r
a0.push(new A.e(s))}return new A.a_(a0,o,null,null)}},
B(a,b){if(b==null)return!1
return b instanceof A.aW&&b.a===this.a&&b.b===this.b&&b.c===this.c},
gn(a){return A.aS(this.a,this.b,this.c,B.h)},
h(a){return"Shape("+this.b+")"}}
A.bI.prototype={
bz(a){var s,r,q,p,o,n,m=this,l=4
m.r=!0
try{q=$.d
if(q==null)q=$.d=A.m()
q.a.$2("f2d_world_step",t.r.a(A.h([m.a,a,A.j(l)],t.a)))}finally{m.r=!1
q=m.w
if(q.length!==0){p=A.dm(q,t.M)
s=p
B.c.M(q)
for(q=s,o=q.length,n=0;n<q.length;q.length===o||(0,A.S)(q),++n){r=q[n]
r.$0()}}}},
v(a){var s,r,q,p,o,n,m,l,k,j,i,h,g,f
if(this.r)A.A(A.C("Cannot create a body while the world is stepping. Collision callbacks run inside step(); create after step() returns instead. Destroy operations are deferred automatically."))
s=a==null?A.K(0,!0,!1,!1,null,null,B.d,B.j):a
r=$.d
if(r==null)r=$.d=A.m()
q=s.b.a
p=q[0]
q=q[1]
o=s.c
n=s.d.a
m=n[0]
n=n[1]
l=r.a
k=l.cu(null)
j=s.Q?1:0
i=s.at?1:0
h=s.ax?1:0
g=l.x
g===$&&A.l("_scratch")
l.$2("f2d_create_body",t.r.a(A.h([this.a,s.a.a,p,q,o.a,o.b,m,n,s.e,0,0,1,0.05,k,j,1,i,h,1,0,g],t.a)))
f=r.L()
return new A.V(this,f.a,f.b)},
b5(){return this.v(null)},
b7(a){var s,r,q,p,o,n,m,l,k,j,i,h,g=this
if(g.r)A.A(A.C(u.e))
s=$.d
if(s==null)s=$.d=A.m()
r=a.a
q=a.b
p=a.e.a
o=p[0]
p=p[1]
n=a.f.a
m=n[0]
n=n[1]
l=s.a
k=l.x
k===$&&A.l("_scratch")
l.$2("f2d_create_distance_joint",t.r.a(A.h([g.a,r.b,r.c,q.b,q.c,o,p,m,n,a.r,1,a.x,a.y,0,0,1e5,0,0,0,0,k],t.a)))
j=s.L()
i=j.a
h=j.b
g.aW(i,h,a)
return new A.dj(g,i,h)},
b8(a){var s,r,q,p,o,n,m,l,k,j,i,h,g=this
if(g.r)A.A(A.C(u.e))
s=$.d
if(s==null)s=$.d=A.m()
r=a.a
q=a.b
p=a.e.a
o=p[0]
p=p[1]
n=a.f.a
m=n[0]
n=n[1]
l=s.a
k=l.x
k===$&&A.l("_scratch")
l.$2("f2d_create_revolute_joint",t.r.a(A.h([g.a,r.b,r.c,q.b,q.c,o,p,m,n,0,0,0,0,0,0,0,0,0,0,0,0.25,0,k],t.a)))
j=s.L()
i=j.a
h=j.b
g.aW(i,h,a)
return new A.dx(g,i,h)},
aW(a,b,c){},
bn(a){var s,r,q,p,o,n=$.d
if(n==null)n=$.d=A.m()
s=a.a.a
r=a.b.a
q=n.cr(this.a,s[0],s[1],r[0],r[1],1,-1)
r=A.h([],t.w)
for(p=0;n=q.length,p<n;p+=2){s=q[p]
o=p+1
if(!(o<n))return A.f(q,o)
r.push(new A.aW(this,s,q[o]))}return r},
c3(a){var s=new A.ec(),r=$.d
if(r==null)r=$.d=A.m()
r.cq(this.a,new A.bS([!1,!1,new A.e3(a),!1,!1,!1,!1,!1,!1,!1,!1,!0,!1,new A.e4(a),new A.e5(a,s),new A.e6(a),a.b,new A.e7(a),new A.e8(a),new A.e9(a,s),new A.ea(a),new A.eb(a),null]))}}
A.ec.prototype={
$1(a){var s,r,q,p,o
t.o.a(a)
s=A.h([],t.F)
for(r=0;q=a.length,r<q;r+=2){p=a[r]
o=r+1
if(!(o<q))return A.f(a,o)
o=a[o]
q=new Float32Array(2)
q[1]=o
q[0]=p
s.push(new A.e(q))}return s},
$S:24}
A.e5.prototype={
$2(a,b){var s=this.a
s.aT(t.Y.a(this.b.$1(t.o.a(a))))
s.N(b)
return null},
$S:17}
A.e9.prototype={
$7(a,b,c,d,e,f,g){var s
t.o.a(e)
s=new A.e(new Float32Array(2))
s.k(a,b)
return this.a.bd(new A.aZ(s,new A.al(c,d)),this.b.$1(e),f,g)},
$S:18}
A.e3.prototype={
$4(a,b,c,d){var s,r=this.a,q=new Float32Array(2)
new A.e(q).k(a,b)
s=r.ax
s.beginPath()
A.dc(s,"arc",[r.H(q[0]),r.I(q[1]),c*r.CW,0,6.283185307179586],t.H)
r.N(d)
return null},
$S:12}
A.e8.prototype={
$6(a,b,c,d,e,f){var s=new A.e(new Float32Array(2))
s.k(a,b)
return this.a.bc(new A.aZ(s,new A.al(c,d)),e,f)},
$S:11}
A.e7.prototype={
$6(a,b,c,d,e,f){var s,r=new A.e(new Float32Array(2))
r.k(a,b)
s=new A.e(new Float32Array(2))
s.k(c,d)
return this.a.bb(r,s,e,f)},
$S:11}
A.e6.prototype={
$5(a,b,c,d,e){var s,r=new A.e(new Float32Array(2))
r.k(a,b)
s=new A.e(new Float32Array(2))
s.k(c,d)
return this.a.ba(r,s,e)},
$S:21}
A.eb.prototype={
$4(a,b,c,d){new A.e(new Float32Array(2)).k(a,b)
return null},
$S:22}
A.e4.prototype={
$4(a,b,c,d){var s,r=this.a,q=new Float32Array(2)
new A.e(q).k(a,b)
s=r.ax
s.beginPath()
A.dc(s,"arc",[r.H(q[0]),r.I(q[1]),c/2,0,6.283185307179586],t.H)
s.fillStyle=r.a5(d,0.9)
s.fill()
return null},
$S:12}
A.ea.prototype={
$4(a,b,c,d){var s,r=this.a,q=new Float32Array(2)
new A.e(q).k(a,b)
s=r.ax
s.fillStyle=r.a5(15067637,0.8)
s.font="12px ui-sans-serif, system-ui"
s.fillText(c,r.H(q[0]),r.I(q[1]))
return null},
$S:23}
A.a0.prototype={
bP(){var s=this.a,r=s.x
r===$&&A.l("_scratch")
s=s.d
s===$&&A.l("_f32")
return new A.N(A.c(s[B.b.j(r,4)]),A.c(s[B.b.j(r+4,4)]))},
L(){var s,r=this.a,q=r.x
q===$&&A.l("_scratch")
r=r.f
r===$&&A.l("_i32")
s=A.j(r[B.b.j(q,4)])
q=A.j(r[B.b.j(q+4,4)])
return new A.N(s,q<0?q+4294967296:q)},
ap(a){var s,r,q,p=this.a,o=p.x
o===$&&A.l("_scratch")
s=o+128
o=a.a
r=A.by(o[0])
q=A.by(o[11])
p.a0(s,o[7])
p.a0(s+4,o[12])
p.a0(s+8,o[13])
p.a0(s+12,o[14])
p.D(s+16,o[16])
p.D(s+20,o[1])
p.a0(s+24,o[2])
p.D(s+28,r.a)
p.D(s+32,r.b)
p.D(s+36,q.a)
p.D(s+40,q.b)
p.D(s+44,o[8])
p.D(s+48,0)
p.D(s+52,0)
p.D(s+56,0)
p.D(s+60,0)
p.D(s+64,0)
p.D(s+68,1)
p.D(s+72,1)},
cr(a,b,c,d,e,f,g){var s,r,q,p,o,n,m,l=null,k=null,j=A.by(f)
l=j.a
k=j.b
s=null
r=null
p=A.by(g)
s=p.a
r=p.b
o=A.h([],t.t)
n=this.a
m=n.c
q=m.b
m.sbm(new A.dw(o))
try{n.$2("f2d_world_overlap_aabb",t.r.a(A.h([a,b,c,d,e,l,k,s,r],t.a)))}finally{m.sbm(q)}return o},
cs(a,b){this.b.aH(0,(a&65535)-1)
this.a.$2("f2d_world_set_custom_filter",t.r.a(A.h([a,0],t.a)))},
ct(a,b){this.c.aH(0,(a&65535)-1)
this.a.$2("f2d_world_set_pre_solve",t.r.a(A.h([a,0],t.a)))},
cq(a,b){var s,r,q,p,o,n,m,l,k,j,i=null
t.G.a(b)
q=this.a
p=q.c
s=p.e
o=b.a
p.e=new A.cR(o[14],o[19],o[2],o[18],o[17],o[15],o[21],o[13],o[20])
try{r=o[22]
n=r!=null?1:0
m=r
m=m==null?i:m.a[0]
if(m==null)m=0
l=r
l=l==null?i:l.a[1]
if(l==null)l=0
k=r
k=k==null?i:k.a[2]
if(k==null)k=0
j=r
j=j==null?i:j.a[3]
if(j==null)j=0
o=o[16]?1:0
q.$2("f2d_world_draw",t.r.a(A.h([a,n,m,l,k,j,o,1,0,0,0,0,0,0,0,0,0,0,0],t.a)))}finally{p.sc2(s)}}}
A.dw.prototype={
$2(a,b){var s=this.a
B.c.m(s,a)
B.c.m(s,b)
return!0},
$S:48}
A.dI.prototype={
sbm(a){this.b=t.d3.a(a)},
sc2(a){this.e=t.c8.a(a)}}
A.cR.prototype={}
A.cS.prototype={
am(){var s=this,r=t.h.a(s.b.buffer),q=v.G
s.d=A.t(new q.Float32Array(r))
A.t(new q.Float64Array(r))
s.f=A.t(new q.Int32Array(r))
q=A.t(new q.Uint8Array(r))
s.r=q
s.w=A.j(q.length)},
aS(a){var s=this.at,r=s.O(0,"malloc")
if(r==null){r=this.a.malloc
r.toString
t.g.a(r)
s.p(0,"malloc",r)
s=r}else s=r
s=s.call(null,a)
s.toString
return B.f.W(A.c(s))},
$2(a,b){var s,r,q,p,o,n=this
A.Q(a)
t.r.a(b)
s=n.at
r=s.O(0,a)
if(r==null){q=n.a[a]
q.toString
t.g.a(q)
s.p(0,a,q)
r=q}s=[]
for(q=b.length,p=0;p<b.length;b.length===q||(0,A.S)(b),++p)s.push(b[p])
o=r.apply.apply(r,[null,s])
if(A.j(t.h.a(n.b.buffer).byteLength)!==n.w)n.am()
A:{s=0
if(o==null)break A
if(typeof o=="number"){s=o
break A}break A}return s},
b2(a){var s,r,q=this
if(a>q.z){if(q.y!==0){s=q.at
r=s.O(0,"free")
if(r==null){r=q.a.free
r.toString
t.g.a(r)
s.p(0,"free",r)
s=r}else s=r
s.call(null,q.y)}s=a*2
q.z=s
q.y=q.aS(s)}return q.y},
a0(a,b){var s=this.d
s===$&&A.l("_f32")
s[B.b.j(a,4)]=b
return b},
D(a,b){var s=this.f
s===$&&A.l("_i32")
s[B.b.j(a,4)]=b
return b},
aG(a,b){var s,r,q=B.b.j(a,4),p=A.h([],t.u)
for(s=0;s<b;++s){r=this.d
r===$&&A.l("_f32")
p.push(A.c(r[q+s]))}return p},
aJ(a,b){var s,r,q
t.o.a(b)
s=B.b.j(a,4)
for(r=0;r<b.length;++r){q=this.d
q===$&&A.l("_f32")
q[s+r]=b[r]}},
cu(a){return 0},
ck(a){var s,r,q,p,o
if(a===0)return""
s=A.h([],t.t)
for(r=a;;r=p){q=this.r
q===$&&A.l("_u8")
p=r+1
o=A.j(q[r])
if(o===0)break
B.c.m(s,o)}t.L.a(s)
return B.aj.c_(s)}}
A.e0.prototype={
$2(a,b){return this.a[a]=b},
$S:25}
A.dK.prototype={
$7(a,b,c,d,e,f,g){A.c(a)
A.c(b)
A.c(c)
A.c(d)
A.c(e)
A.c(f)
A.c(g)
this.a.q().toString
return 1},
$C:"$7",
$R:7,
$S:26}
A.dL.prototype={
$2(a,b){var s
A.c(a)
A.c(b)
s=this.a.q().c.b
return s==null||s.$2(A.j(a),A.j(b))?1:0},
$S:27}
A.dM.prototype={
$4(a,b,c,d){var s
A.c(a)
A.c(b)
A.c(c)
A.c(d)
s=this.a.q().c.c
return s==null||s.$4(A.j(a),A.j(b),A.j(c),A.j(d))?1:0},
$C:"$4",
$R:4,
$S:10}
A.dT.prototype={
$6(a,b,c,d,e,f){var s
A.c(a)
A.c(b)
A.c(c)
A.c(d)
A.c(e)
A.c(f)
s=this.a.q().c.d
return s==null||s.$6(A.j(a),A.j(b),A.j(c),A.j(d),e,f)?1:0},
$C:"$6",
$R:6,
$S:29}
A.dU.prototype={
$3(a,b,c){var s,r
A.c(a)
A.c(b)
A.c(c)
s=this.a
r=s.q().c.e
if(r!=null){s=s.q().aG(A.j(a),A.j(b)*2)
A.j(c)
r.a.$2(s,c)}},
$C:"$3",
$R:3,
$S:30}
A.dV.prototype={
$8(a,b,c,d,e,f,g,h){var s,r
A.c(a)
A.c(b)
A.c(c)
A.c(d)
A.c(e)
A.c(f)
A.c(g)
A.c(h)
s=this.a
r=s.q().c.e
if(r!=null){s=s.q().aG(A.j(e),A.j(f)*2)
A.j(h)
r.b.$7(a,b,c,d,s,g,h)}},
$C:"$8",
$R:8,
$S:31}
A.dW.prototype={
$4(a,b,c,d){var s
A.c(a)
A.c(b)
A.c(c)
A.c(d)
s=this.a.q().c.e
if(s!=null){A.j(d)
s.c.$4(a,b,c,d)}},
$C:"$4",
$R:4,
$S:3}
A.dX.prototype={
$6(a,b,c,d,e,f){var s
A.c(a)
A.c(b)
A.c(c)
A.c(d)
A.c(e)
A.c(f)
s=this.a.q().c.e
if(s!=null){A.j(f)
s.d.$6(a,b,c,d,e,f)}},
$C:"$6",
$R:6,
$S:9}
A.dY.prototype={
$6(a,b,c,d,e,f){var s
A.c(a)
A.c(b)
A.c(c)
A.c(d)
A.c(e)
A.c(f)
s=this.a.q().c.e
if(s!=null){A.j(f)
s.e.$6(a,b,c,d,e,f)}},
$C:"$6",
$R:6,
$S:9}
A.dZ.prototype={
$5(a,b,c,d,e){var s
A.c(a)
A.c(b)
A.c(c)
A.c(d)
A.c(e)
s=this.a.q().c.e
if(s!=null){A.j(e)
s.f.$5(a,b,c,d,e)}},
$C:"$5",
$R:5,
$S:34}
A.e_.prototype={
$4(a,b,c,d){var s
A.c(a)
A.c(b)
A.c(c)
A.c(d)
s=this.a.q().c.e
if(s!=null)s.r.$4(a,b,c,d)},
$C:"$4",
$R:4,
$S:3}
A.dN.prototype={
$4(a,b,c,d){var s
A.c(a)
A.c(b)
A.c(c)
A.c(d)
s=this.a.q().c.e
if(s!=null){A.j(d)
s.w.$4(a,b,c,d)}},
$C:"$4",
$R:4,
$S:3}
A.dO.prototype={
$4(a,b,c,d){var s,r
A.c(a)
A.c(b)
A.c(c)
A.c(d)
s=this.a
r=s.q().c.e
if(r!=null){s=s.q().ck(A.j(c))
A.j(d)
r.x.$4(a,b,s,d)}},
$C:"$4",
$R:4,
$S:3}
A.dP.prototype={
$1(a){A.c(a)
this.a.q().am()},
$S:35}
A.dQ.prototype={
$3(a,b,c){var s,r,q,p,o,n
A.c(a)
A.c(c)
s=A.c(v.G.performance.now())*1e6
r=B.f.bf(B.f.G(s,4294967296))
q=B.f.bf(s/4294967296)
p=B.b.j(A.j(c),4)
o=this.a
n=o.q().f
n===$&&A.l("_i32")
n[p]=r
o=o.q().f
o===$&&A.l("_i32")
o[p+1]=q
return 0},
$C:"$3",
$R:3,
$S:36}
A.dR.prototype={
$4(a,b,c,d){var s
A.c(a)
A.c(b)
A.c(c)
A.c(d)
s=this.a.q().f
s===$&&A.l("_i32")
s[B.b.j(A.j(d),4)]=0
return 0},
$C:"$4",
$R:4,
$S:10}
A.dS.prototype={
$1(a){throw A.i(A.C("box2d.wasm exited with code "+A.iG(A.c(a))))},
$S:37}
A.e.prototype={
k(a,b){var s=this.a
s.$flags&2&&A.ar(s)
s[1]=b
s[0]=a},
a9(a){var s=a.a,r=this.a,q=s[1]
r.$flags&2&&A.ar(r)
r[1]=q
r[0]=s[0]},
h(a){var s=this.a
return"["+A.r(s[0])+","+A.r(s[1])+"]"},
B(a,b){var s,r,q
if(b==null)return!1
if(b instanceof A.e){s=this.a
r=s[1]
q=b.a
s=r===q[1]&&s[0]===q[0]}else s=!1
return s},
gn(a){return A.h2(this.a)},
a1(a,b){var s,r=new Float32Array(2),q=new A.e(r)
q.a9(this)
s=b.a
r[1]=r[1]-s[1]
r[0]=r[0]-s[0]
return q},
a8(a,b){var s=new A.e(new Float32Array(2))
s.a9(this)
s.m(0,b)
return s},
gA(a){var s=this.a,r=s[1]
s=s[0]
return Math.sqrt(r*r+s*s)},
gcb(){var s=this.a,r=s[1]
s=s[0]
return r*r+s*s},
m(a,b){var s=b.a,r=this.a,q=r[1],p=s[1]
r.$flags&2&&A.ar(r)
r[1]=q+p
r[0]=r[0]+s[0]}}
A.fp.prototype={}
A.bK.prototype={}
A.cZ.prototype={}
A.bL.prototype={$iiU:1}
A.er.prototype={
$1(a){return this.a.$1(A.t(a))},
$S:1}
A.cd.prototype={
H(a){return this.cx/2+(a-this.ay.a[0])*this.CW},
I(a){return this.cy/2-(a-this.ay.a[1])*this.CW},
a5(a,b){return"rgba("+(B.b.R(a,16)&255)+", "+(B.b.R(a,8)&255)+", "+(a&255)+", "+A.r(b)+")"},
N(a){var s=this.ax
s.strokeStyle=this.a5(a,0.95)
s.lineWidth=1.6
s.stroke()},
ah(a){var s=this.ax
s.fillStyle=this.a5(a,0.28)
s.fill()},
aT(a){var s,r,q,p,o,n,m,l=this
t.Y.a(a)
s=l.ax
s.beginPath()
for(r=0;r<a.length;++r){q=a[r].a
p=l.cx
o=l.ay
n=l.CW
p/=2
o=o.a
m=l.cy/2
if(r===0)s.moveTo(p+(q[0]-o[0])*n,m-(q[1]-o[1])*n)
else s.lineTo(p+(q[0]-o[0])*n,m-(q[1]-o[1])*n)}s.closePath()},
bd(a,b,c,d){var s,r,q,p,o,n
t.Y.a(b)
s=A.h([],t.F)
for(r=b.length,q=a.b,p=a.a,o=0;o<b.length;b.length===r||(0,A.S)(b),++o){n=q.V(b[o])
n.m(0,p)
s.push(n)}this.aT(s)
this.ah(d)
this.N(d)},
bc(a,b,c){var s,r,q,p=this,o=p.ax
o.beginPath()
s=a.a.a
A.dc(o,"arc",[p.H(s[0]),p.I(s[1]),b*p.CW,0,6.283185307179586],t.H)
p.ah(c)
p.N(c)
r=new A.e(new Float32Array(2))
r.k(b,0)
q=a.bV(r)
o.beginPath()
o.moveTo(p.H(s[0]),p.I(s[1]))
s=q.a
o.lineTo(p.H(s[0]),p.I(s[1]))
p.N(c)},
bb(a,b,c,d){var s,r,q,p=this,o=b.a1(0,a).a,n=Math.atan2(o[1],o[0])
o=p.ax
o.beginPath()
a=a.a
s=-n
r=s+1.5707963267948966
q=t.H
A.dc(o,"arc",[p.H(a[0]),p.I(a[1]),c*p.CW,r,s+4.71238898038469],q)
b=b.a
A.dc(o,"arc",[p.H(b[0]),p.I(b[1]),c*p.CW,s-1.5707963267948966,r],q)
o.closePath()
p.ah(d)
p.N(d)},
ba(a,b,c){var s=this,r=s.ax
r.beginPath()
a=a.a
r.moveTo(s.H(a[0]),s.I(a[1]))
b=b.a
r.lineTo(s.H(b[0]),s.I(b[1]))
s.N(c)}}
A.a7.prototype={
a3(){return"DomEvent."+this.b}}
A.eU.prototype={
$1(a){return this.a.$1(this.b.a(A.t(a)))},
$S:1}
A.cU.prototype={
by(){var s,r,q,p,o,n,m,l,k,j=this
for(s=$.de(),r=v.G,q=t.bU,p=q.i("~(1)?"),q=q.c,o=j.c,n=0;n<7;++n){m=s[n]
l=A.t(A.t(r.document).createElement("button"))
l.textContent=m.a
A.hj(l,"click",p.a(new A.ed(j,m)),!1,q)
o.append(l)}s=t.m
A.b5(B.I,A.t(r.document),new A.ee(j),s)
A.b5(B.J,A.t(r.document),new A.ef(j),s)
q=A.aD(A.t(r.document).getElementById("reset"))
q.toString
A.b5(B.O,q,new A.eg(j),s)
q=j.a
A.b5(B.K,q,j.gbL(),s)
A.b5(B.L,q,j.gbN(),s)
A.b5(B.M,q,new A.eh(j),s)
A.b5(B.N,q,new A.ei(j),s)
k=A.j3().gav()
j.ao(B.c.c6($.de(),new A.ej(j,k),new A.ek()))
A.j(A.t(r.window).requestAnimationFrame(A.c2(j.gaR())))},
ao(a){var s,r,q,p,o,n,m,l,k,j=this,i=null
j.an()
s=j.e
if(s!=null){if(s.r)A.A(A.C("Cannot destroy the world while the world is stepping. Collision callbacks run inside step(); create after step() returns instead. Destroy operations are deferred automatically."))
r=$.d
if(r==null)r=$.d=A.m()
q=s.a
r.cs(q,i)
r=$.d;(r==null?$.d=A.m():r).ct(q,i)
r=$.d
if(r==null)r=$.d=A.m()
r.a.$2("f2d_destroy_world",t.r.a(A.h([q],t.a)))
s.b.M(0)
s.c.M(0)
s.d.M(0)
s.e.M(0)
s.f.M(0)
B.c.M(s.w)}j.r=a
j.f=new A.aU()
s=new A.e(new Float32Array(2))
s.k(0,-10)
s=new A.e2(s)
r=t.bQ
q=t.X
p=A.h([],t.x)
o=$.d
if(o==null)o=$.d=A.m()
s=s.a
n=s.a[0]
s=s.a[1]
s=new A.bI(B.f.W(o.a.$2("f2d_create_world",t.r.a(A.h([n,s,1,1,30,10,3,400,1,1],t.a)))),A.B(r,q),A.B(r,q),A.B(r,q),A.B(r,q),A.B(r,r),p)
a.e.$2(s,j.f)
j.e=s
s=j.d
s===$&&A.l("_draw")
r=new A.e(new Float32Array(2))
r.k(0,a.d)
s.ay=r
s.ch=a.c
j.b.textContent=a.b
s=A.t(A.t(v.G.window).location)
r=a.a
s.hash=A.i3(r.toLowerCase()," ","-")
m=A.t(j.c.querySelectorAll("button"))
for(l=0;l<A.j(m.length);++l){k=A.aD(m.item(l))
s=A.fA(k.textContent)===r?"active":""
k.className=s}},
bJ(a){var s,r,q,p,o,n,m,l=this,k=A.c(a)/1000,j=l.x,i=j===0?0.016666666666666666:B.f.b3(k-j,0,0.1)
l.x=k
j=l.y+=i
s=l.e
if(s!=null){while(j>=0.016666666666666666){j=l.f.b
if(j!=null)j.$1(0.016666666666666666)
s.bz(0.016666666666666666)
j=l.y-=0.016666666666666666}r=l.f.d
if(r!=null){j=l.d
j===$&&A.l("_draw")
j.ay.a9(r.$0())}q=A.c(A.t(v.G.window).devicePixelRatio)
j=l.a
p=B.f.br(A.j(j.clientWidth)*q)
o=B.f.br(A.j(j.clientHeight)*q)
if(A.j(j.width)!==p||A.j(j.height)!==o){j.width=p
j.height=o}n=l.d
n===$&&A.l("_draw")
m=A.j(j.width)
j=A.j(j.height)
n.cx=m
n.cy=j
n.CW=j/n.ch
n.ax.clearRect(0,0,m,j)
l.bR(s)
n.b=!1
s.c3(n)}A.j(A.t(v.G.window).requestAnimationFrame(A.c2(l.gaR())))},
bR(b2){var s,r,q,p,o,n,m,l,k,j,i,h,g,f,e,d,c,b,a,a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,b0=null,b1=this.d
b1===$&&A.l("_draw")
s=b1.ch/2
r=this.a
q=A.j(r.width)
r=B.b.b3(A.j(r.height),1,65536)
p=new A.e(new Float32Array(2))
p.k(s*(q/r)+5,s+5)
for(r=b2.bn(new A.c8(b1.ay.a1(0,p),b1.ay.a8(0,p))),q=r.length,o=t.a,n=t.r,m=t.S,l=t.p,k=t._,j=t.b4,i=0;i<r.length;r.length===q||(0,A.S)(r),++i){h=r[i]
g=h.gb1()
f=g.ga7()
e=$.d
if(e==null){d=$.a6
if(d==null)A.A(A.C(u.j))
e=$.d=new A.a0(d,A.B(m,l),A.B(m,k))}e=e.a
c=e.x
c===$&&A.l("_scratch")
e.$2("f2d_body_get_rotation",n.a(A.h([g.b,g.c,c],o)))
e=e.d
e===$&&A.l("_f32")
c=new A.al(A.c(e[B.b.j(c,4)]),A.c(e[B.b.j(c+4,4)]))
g=h.a.c
e=h.b
b=h.c
if(A.fE(g.O(0,new A.N(e,b)))){g=g.O(0,new A.N(e,b))
g.toString
a=g}else a=6583435
a0=h.gbv()
a1=a0 instanceof A.W
a2=b0
if(a1){a3=a0.a
a4=a0.b
a2=a4}else a3=b0
if(a1){g=c.V(j.a(a3))
g.m(0,f)
b1.bc(new A.aZ(g,c),a2,a)
continue}a1=a0 instanceof A.bd
a5=b0
a2=b0
if(a1){a6=a0.a
a5=a0.b
a4=a0.c
a2=a4}else a6=b0
if(a1){g=c.V(j.a(a6))
g.m(0,f)
c=c.V(j.a(a5))
c.m(0,f)
b1.bb(g,c,a2,a)
continue}g=a0 instanceof A.aV
if(g){a7=a0.a
a8=a0.b}else{a8=b0
a7=a8}if(g){g=c.V(j.a(a7))
g.m(0,f)
a8=c.V(j.a(a8))
a8.m(0,f)
b1.ba(g,a8,a)
continue}a1=a0 instanceof A.a_
a2=b0
if(a1){a9=a0.a
a4=a0.b
a2=a4}else a9=b0
if(a1){a9.toString
b1.bd(new A.aZ(f,c),a9,a2,a)}}},
b0(a){var s,r,q,p,o,n,m,l,k,j=A.t(this.a.getBoundingClientRect()),i=A.c(A.t(v.G.window).devicePixelRatio),h=this.d
h===$&&A.l("_draw")
s=A.j(a.clientX)
r=A.c(j.left)
q=A.j(a.clientY)
p=A.c(j.top)
o=h.ay.a
n=o[0]
m=h.cx
l=h.CW
o=o[1]
h=h.cy
k=new A.e(new Float32Array(2))
k.k(n+((s-r)*i-m/2)/l,o-((q-p)*i-h/2)/l)
return k},
bM(a8){var s,r,q,p,o,n,m,l,k,j,i,h,g,f,e,d,c,b,a,a0,a1,a2,a3,a4=this,a5=u.j,a6=a4.e,a7=a4.f.a
if(a6==null||a7==null)return
s=a4.b0(a8)
r=new A.e(new Float32Array(2))
r.k(0.1,0.1)
r=s.a1(0,r)
q=new A.e(new Float32Array(2))
q.k(0.1,0.1)
p=a6.bn(new A.c8(r,s.a8(0,q)))
for(r=p.length,q=t.a,o=t.r,n=t.S,m=t.p,l=t._,k=s.a,j=0;j<p.length;p.length===r||(0,A.S)(p),++j){i=p[j]
h=i.gb1()
g=$.d
if(g==null){f=$.a6
if(f==null)A.A(A.C(a5))
g=$.d=new A.a0(f,A.B(n,m),A.B(n,l))}e=h.b
d=h.c
g=B.f.W(g.a.$2("f2d_body_get_type",o.a(A.h([e,d],q))))
if(!(g>=0&&g<3))return A.f(B.w,g)
if(B.w[g]===B.e){g=$.d
if(g==null){f=$.a6
if(f==null)A.A(A.C(a5))
g=$.d=new A.a0(f,A.B(n,m),A.B(n,l))}g=g.a.$2("f2d_shape_test_point",o.a(A.h([i.b,i.c,k[0],k[1]],q)))!==0}else g=!1
if(g){r=$.d
if(r==null){f=$.a6
if(f==null)A.A(A.C(a5))
r=$.d=new A.a0(f,A.B(n,m),A.B(n,l))}r=r.a.$2("f2d_body_get_mass",o.a(A.h([e,d],q)))
if(a6.r)A.A(A.C(u.e))
g=$.d
if(g==null){f=$.a6
if(f==null)A.A(A.C(a5))
g=$.d=new A.a0(f,A.B(n,m),A.B(n,l))}c=a6.a
b=a7.b
a=a7.c
a0=k[0]
a1=k[1]
k=g.a
g=k.x
g===$&&A.l("_scratch")
k.$2("f2d_create_mouse_joint",o.a(A.h([c,b,a,e,d,a0,a1,6,0.8,1500*r,0,g],q)))
k=k.f
k===$&&A.l("_i32")
a2=A.j(k[B.b.j(g,4)])
a3=A.j(k[B.b.j(g+4,4)])
if(a3<0)a3+=4294967296
a4.w=new A.dq(a6,a2,a3)
r=$.d
if(r==null){f=$.a6
if(f==null)A.A(A.C(a5))
r=$.d=new A.a0(f,A.B(n,m),A.B(n,l))}r.a.$2("f2d_body_set_awake",o.a(A.h([e,d,1],q)))
a4.a.setPointerCapture(A.j(a8.pointerId))
return}}},
bO(a){var s,r,q,p,o,n,m,l,k=this.w
if(k!=null){s=$.d
if(s==null)s=$.d=A.m()
s=s.a.$2("f2d_joint_is_valid",t.r.a(A.h([k.b,k.c],t.a)))!==0}else s=!1
if(s){s=this.b0(a)
r=$.d
if(r==null)r=$.d=A.m()
q=k.b
p=k.c
s=s.a
o=t.a
n=t.r
r.a.$2("f2d_mouse_joint_set_target",n.a(A.h([q,p,s[0],s[1]],o)))
s=$.d
if(s==null)s=$.d=A.m()
r=s.a
m=r.x
m===$&&A.l("_scratch")
r.$2("f2d_joint_get_body_b",n.a(A.h([q,p,m],o)))
l=s.L()
s=$.d
if(s==null)s=$.d=A.m()
s.a.$2("f2d_body_set_awake",n.a(A.h([l.a,l.b,1],o)))}},
an(){var s,r,q=this.w,p=!1
if(q!=null){s=$.d
p=s==null?$.d=A.m():s
p=p.a.$2("f2d_joint_is_valid",t.r.a(A.h([q.b,q.c],t.a)))!==0
r=q}else r=null
if(p)r.b9()
this.w=null}}
A.ed.prototype={
$1(a){return this.a.ao(this.b)},
$S:1}
A.ee.prototype={
$1(a){var s=this.a.f.c
return s==null?null:s.$2$down(A.Q(a.key),!0)},
$S:1}
A.ef.prototype={
$1(a){var s=this.a.f.c
return s==null?null:s.$2$down(A.Q(a.key),!1)},
$S:1}
A.eg.prototype={
$1(a){var s=this.a
return s.ao(s.r)},
$S:1}
A.eh.prototype={
$1(a){return this.a.an()},
$S:1}
A.ei.prototype={
$1(a){return this.a.an()},
$S:1}
A.ej.prototype={
$1(a){t.az.a(a)
return A.i3(a.a.toLowerCase()," ","-")===this.b},
$S:40}
A.ek.prototype={
$0(){return B.c.gbe($.de())},
$S:41}
A.aU.prototype={
saE(a){this.b=t.W.a(a)},
sci(a){this.c=t.aL.a(a)},
sbY(a){this.d=t.I.a(a)}}
A.ab.prototype={}
A.fe.prototype={
$2(a,b){var s,r,q,p,o,n,m,l,k
b.a=A.eT(a,40)
for(s=0,r=0;r<14;++r){q=14-r
p=0.55+r*1.05
for(o=(q-1)/2,n=0;n<q;++n,s=l){m=new Float32Array(2)
m[1]=p
m[0]=(n-o)*1.1
m=a.v(A.K(0,!0,!1,!1,null,new A.e(m),B.d,B.e))
l=s+1
k=B.k[B.b.G(s,6)]
m.C(new A.a_(null,0,0.5,0.5),A.P(1,new A.H(0.6,0,k),k))}}},
$S:2}
A.ff.prototype={
$2(a,b){var s,r,q,p,o,n,m,l,k,j,i,h,g={}
b.a=A.eT(a,45)
g.a=10
g.b=0
s=new A.fl(g,a)
for(r=0;r<25;++r){q=r*1.5-18.75
s.$3$horizontal(q,0.5,!1)
s.$3$horizontal(q,1.1,!0)}for(p=1;p<25;++p){if(p>3)g.a*=0.8
o=0.5+1.386*p
n=25-p
for(m=o+0.6,l=o-0.6,k=n-1,j=1.5*n/2,i=o-0.2,r=0;r<n;++r){q=r*1.5-j
g.a*=2.5
if(r===0)s.$3$horizontal(q-1.25+0.1,i,!1)
if(r===k)s.$3$horizontal(q+1.25-0.1,i,!1)
g.a/=2.5
s.$3$horizontal(q,o,!1)
s.$3$horizontal(q,m,!0)
s.$3$horizontal(q,l,!0)}}h=B.m.cf()?1:-1
m=B.m.K()
l=B.m.K()
g.c=0
g.d=!1
b.saE(new A.fd(g,a,h,8+m*16,42+l*18))},
$S:2}
A.fl.prototype={
$3$horizontal(a,b,c){var s,r,q,p,o,n=new A.e(new Float32Array(2))
n.k(a,b)
n=this.b.v(A.K(0,!0,!1,!1,null,n,c?new A.al(Math.cos(1.5707963267948966),Math.sin(1.5707963267948966)):B.d,B.e))
s=A.bw(0.1,0.5,0)
r=this.a
q=r.b
p=B.k[B.b.G(q,6)]
o=r.a
r.b=q+1
n.C(s,A.P(o,new A.H(0.1,0.65,p),p))},
$S:43}
A.fd.prototype={
$1(a){var s,r=this,q=r.a,p=q.c+=a
if(q.d||p<2)return
q.d=!0
q=r.c
p=new A.e(new Float32Array(2))
p.k(-q*45,r.d)
s=new A.e(new Float32Array(2))
s.k(q*r.e,0)
p=r.b.v(A.K(0,!0,!1,!0,s,p,B.d,B.e))
q=new A.e(new Float32Array(2))
p.C(new A.W(q,1.4),A.P(10,new A.H(0.6,0,16478597),16478597))},
$S:4}
A.fg.prototype={
$2(a,b){var s,r,q,p,o,n,m,l,k=null,j=new Float32Array(2),i=$.d
if(i==null)i=$.d=A.m()
i.a.$2("f2d_world_set_gravity",t.r.a(A.h([a.a,j[0],j[1]],t.a)))
s=a.b5()
b.a=s
j=A.h([],t.F)
for(r=0;r<40;++r){i=-6.283185307179586*r/40
q=Math.cos(i)
i=Math.sin(i)
p=new Float32Array(2)
p[1]=15*i
p[0]=15*q
j.push(new A.e(p))}s.b6(A.fW(!0,j))
o=a.v(A.K(0.8,!0,!1,!1,k,k,B.d,B.o))
o.C(A.bw(11,0.4,0),A.hQ(10980346,k,k))
b.saE(new A.fc(o))
n=new A.bQ()
n.aa(1)
for(r=0;r<60;++r){m=n.K()*2*3.141592653589793
l=n.K()*10+2
j=Math.cos(m)
i=Math.sin(m)
q=new Float32Array(2)
q[1]=l*i
q[0]=l*j
q=a.v(A.K(0,!0,!1,!1,k,new A.e(q),B.d,B.e))
j=n.K()
i=new A.e(new Float32Array(2))
p=B.k[B.b.G(r,6)]
q.C(new A.W(i,0.35+j*0.35),A.P(1,new A.H(0.1,0.7,p),p))}},
$S:2}
A.fc.prototype={
$1(a){var s=this.a,r=$.d
if(r==null)r=$.d=A.m()
r.a.$2("f2d_body_set_awake",t.r.a(A.h([s.b,s.c,1],t.a)))
return!0},
$S:4}
A.fh.prototype={
$2(a,b){var s,r,q,p,o,n,m,l,k=null
b.a=A.eT(a,18)
for(s=[-18,18],r=0;r<2;++r){q=s[r]
p=new Float32Array(2)
p[1]=12
p[0]=q
a.v(A.K(0,!0,!1,!1,k,new A.e(p),B.d,B.j)).C(new A.a_(k,0,0.5,14),A.P(1,new A.H(0.6,0,6583435),6583435))}o=new A.bQ()
o.aa(7)
for(n=0;n<320;++n){s=o.K()
p=o.K()
m=new Float32Array(2)
m[1]=6+p*20
m[0]=s*30-15
m=a.v(A.K(0,!0,!1,!1,k,new A.e(m),B.d,B.e))
s=o.K()
p=new A.e(new Float32Array(2))
l=B.k[B.b.G(n,6)]
m.C(new A.W(p,0.25+s*0.5),A.P(1,new A.H(0.2,0.35,l),l))}},
$S:2}
A.fi.prototype={
$2(a,b){var s,r,q,p,o,n,m,l
b.a=A.eT(a,40)
s=new Float32Array(2)
new A.e(s).k(0,12)
r=A.h([],t.e)
for(q=0;q<24;++q){p=6.283185307179586*q/24
o=Math.cos(p)
p=Math.sin(p)
n=new Float32Array(2)
n[1]=3.5*p
n[0]=3.5*o
p=new Float32Array(2)
o=new A.e(p)
p[1]=s[1]
p[0]=s[0]
o.m(0,new A.e(n))
o=a.v(A.K(0,!0,!0,!1,null,o,B.d,B.e))
p=new A.e(new Float32Array(2))
o.C(new A.W(p,0.6),A.P(1,new A.H(0.9,0,3462041),3462041))
r.push(o)}m=new A.fm()
for(q=0;q<24;q=l){p=r.length
if(!(q<p))return A.f(r,q)
o=r[q]
l=q+1
n=l%24
if(!(n<p))return A.f(r,n)
a.b7(m.$2(o,r[n]))
n=r.length
if(!(q<n))return A.f(r,q)
o=r[q]
p=(q+12)%24
if(!(p<n))return A.f(r,p)
a.b7(m.$2(o,r[p]))}},
$S:2}
A.fm.prototype={
$2(a,b){var s=Math.sqrt(a.ga7().a1(0,b.ga7()).gcb()),r=new A.e(new Float32Array(2)),q=new A.e(new Float32Array(2))
return new A.aM(r,q,s,!0,6,0.4,a,b,!1,null)},
$S:44}
A.fj.prototype={
$2(a,b){var s,r,q,p,o,n,m,l,k,j=null,i=6583435,h=new A.e(new Float32Array(2))
h.k(-14,6)
s=a.v(A.K(0,!0,!1,!1,j,h,B.d,B.j))
s.C(A.bw(0.6,6,0),A.P(1,new A.H(0.6,0,i),i))
h=new A.e(new Float32Array(2))
h.k(14,6)
r=a.v(A.K(0,!0,!1,!1,j,h,B.d,B.j))
r.C(A.bw(0.6,6,0),A.P(1,new A.H(0.6,0,i),i))
b.a=s
q=new A.e(new Float32Array(2))
q.k(0.6,5.6)
for(p=s,o=0;o<20;++o,p=n){h=new Float32Array(2)
h[1]=11.6
h[0]=-13.4+26.8*(o+0.5)/20
n=a.v(A.K(0,!0,!1,!1,j,new A.e(h),B.d,B.e))
n.C(new A.a_(j,0,0.62,0.15),A.P(1,new A.H(0.8,0,16498468),16498468))
h=new Float32Array(2)
h[1]=0
h[0]=-0.67
a.b8(A.h8(p,n,q,new A.e(h)))
h=new Float32Array(2)
q=new A.e(h)
h[1]=0
h[0]=0.67}h=new A.e(new Float32Array(2))
h.k(-0.6,5.6)
a.b8(A.h8(p,r,q,h))
m=new A.bQ()
m.aa(3)
for(o=0;o<8;++o){h=m.K()
l=new Float32Array(2)
l[1]=16+o*1.5
l[0]=h*16-8
l=a.v(A.K(0,!0,!1,!1,j,new A.e(l),B.d,B.e))
if((o&1)===0){h=new A.e(new Float32Array(2))
h=new A.W(h,0.7)}else h=new A.a_(j,0,0.6,0.6)
k=B.k[B.b.G(o,6)]
l.C(h,A.P(1,new A.H(0.4,0,k),k))}},
$S:2}
A.fk.prototype={
$2(b1,b2){var s,r,q,p,o,n,m,l,k,j,i,h,g,f,e,d,c,b,a,a0,a1,a2,a3,a4,a5,a6,a7,a8=null,a9={},b0=b1.b5()
b2.a=b0
s=A.h([],t.F)
for(r=400;r>=-30;r-=2){q=Math.sin(r/9)
p=new Float32Array(2)
p[1]=q*1.8+r*0.015
p[0]=r
s.push(new A.e(p))}b0.b6(A.fW(!1,s))
s=new A.e(new Float32Array(2))
s.k(0,4)
o=b1.v(A.K(0,!1,!1,!1,a8,s,B.d,B.e))
o.C(A.bw(1.4,0.35,0),A.hQ(8166655,a8,a8))
n=new A.fn(b1)
s=A.h([],t.V)
for(q=[-1,1],p=t.a,m=t.r,l=b1.a,k=o.b,j=o.c,i=t.S,h=t.p,g=t._,f=0;f<2;++f){r=q[f]
e=n.$1(r)
d=new Float32Array(2)
d[1]=-0.35
d[0]=r
c=new A.e(new Float32Array(2))
b=new Float32Array(2)
a=new A.e(b)
b[1]=1
b[0]=0
b=a
if(b1.r)A.A(A.C(u.e))
a=$.d
if(a==null){a0=$.a6
if(a0==null)A.A(A.C(u.j))
a=$.d=new A.a0(a0,A.B(i,h),A.B(i,g))}a1=e.b
e=e.c
a2=d[0]
d=d[1]
c=c.a
a3=c[0]
c=c[1]
b=b.a
a4=b[0]
b=b[1]
a=a.a
a5=a.x
a5===$&&A.l("_scratch")
a.$2("f2d_create_wheel_joint",m.a(A.h([l,k,j,a1,e,a2,d,a3,c,a4,b,1,4,0.7,0,0,0,1,40,0,0,a5],p)))
a=a.f
a===$&&A.l("_i32")
a6=A.j(a[B.b.j(a5,4)])
a7=A.j(a[B.b.j(a5+4,4)])
if(a7<0)a7+=4294967296
s.push(new A.cT(b1,a6,a7))}a9.a=0
b2.sbY(new A.f9(o))
b2.sci(new A.fa(a9))
b2.saE(new A.fb(a9,s))},
$S:2}
A.fn.prototype={
$1(a){var s,r=new A.e(new Float32Array(2))
r.k(a,3.5)
r=this.a.v(A.K(0,!1,!1,!1,null,r,B.d,B.e))
s=new A.e(new Float32Array(2))
r.C(new A.W(s,0.45),A.P(1,new A.H(1.6,0,16478597),16478597))
return r},
$S:45}
A.f9.prototype={
$0(){var s=this.a.ga7(),r=new A.e(new Float32Array(2))
r.k(0,3)
return s.a8(0,r)},
$S:46}
A.fa.prototype={
$2$down(a,b){var s
if(a==="ArrowRight"){s=b?-30:0
this.a.a=s}else if(a==="ArrowLeft"){s=b?30:0
this.a.a=s}},
$S:47}
A.fb.prototype={
$1(a){var s,r,q,p,o,n,m,l,k,j,i,h,g
for(s=this.b,r=s.length,q=t.a,p=t.r,o=this.a,n=t.S,m=t.p,l=t._,k=0;k<s.length;s.length===r||(0,A.S)(s),++k){j=s[k]
i=o.a
h=$.d
if(h==null){g=$.a6
if(g==null)A.A(A.C(u.j))
h=$.d=new A.a0(g,A.B(n,m),A.B(n,l))}h.a.$2("f2d_wheel_joint_set_motor_speed",p.a(A.h([j.b,j.c,i],q)))}},
$S:4};(function aliases(){var s=J.aj.prototype
s.bA=s.h})();(function installTearOffs(){var s=hunkHelpers._static_1,r=hunkHelpers._static_0,q=hunkHelpers._instance_0u,p=hunkHelpers._instance_1u
s(A,"kr","j5",6)
s(A,"ks","j6",6)
s(A,"kt","j7",6)
r(A,"hS","kl",0)
q(A.bo.prototype,"gc1","b9",0)
var o
p(o=A.cU.prototype,"gaR","bJ",4)
p(o,"gbL","bM",1)
p(o,"gbN","bO",1)})();(function inheritance(){var s=hunkHelpers.mixin,r=hunkHelpers.inherit,q=hunkHelpers.inheritMany
r(A.p,null)
q(A.p,[A.fr,J.co,A.bz,J.c9,A.u,A.dy,A.cu,A.L,A.ao,A.a5,A.aP,A.be,A.cr,A.ai,A.dD,A.du,A.bh,A.bT,A.eG,A.bp,A.dl,A.ep,A.a1,A.d0,A.eL,A.eJ,A.cV,A.U,A.cX,A.az,A.E,A.cW,A.bB,A.d3,A.c1,A.x,A.bZ,A.ch,A.ck,A.eN,A.eq,A.cE,A.bA,A.es,A.a8,A.v,A.d4,A.J,A.c_,A.dF,A.d2,A.dt,A.eD,A.bQ,A.V,A.ce,A.cl,A.cm,A.H,A.e2,A.dh,A.dz,A.di,A.ay,A.au,A.bo,A.al,A.aZ,A.c8,A.aW,A.bI,A.a0,A.dI,A.cR,A.cS,A.e,A.fp,A.bL,A.cU,A.aU,A.ab])
q(J.co,[J.cq,J.bj,J.bm,J.bl,J.bn,J.bk,J.aO])
q(J.bm,[J.aj,J.n,A.aR,A.bs])
q(J.aj,[J.cF,J.bC,J.a4])
r(J.cp,A.bz)
r(J.dk,J.n)
q(J.bk,[J.bi,J.cs])
q(A.u,[A.aw,A.ad,A.ct,A.cO,A.cK,A.d_,A.ca,A.a3,A.cD,A.bE,A.cN,A.aX,A.ci])
q(A.a5,[A.aC,A.b_])
q(A.aC,[A.bR,A.bS])
r(A.N,A.b_)
r(A.b0,A.aP)
r(A.bD,A.b0)
r(A.bf,A.bD)
r(A.bg,A.be)
q(A.ai,[A.cg,A.cf,A.cM,A.f_,A.f1,A.em,A.el,A.eR,A.eB,A.dA,A.eI,A.f7,A.f8,A.ec,A.e9,A.e3,A.e8,A.e7,A.e6,A.eb,A.e4,A.ea,A.dK,A.dM,A.dT,A.dU,A.dV,A.dW,A.dX,A.dY,A.dZ,A.e_,A.dN,A.dO,A.dP,A.dQ,A.dR,A.dS,A.er,A.eU,A.ed,A.ee,A.ef,A.eg,A.eh,A.ei,A.ej,A.fl,A.fd,A.fc,A.fn,A.fa,A.fb])
q(A.cg,[A.dv,A.f0,A.eS,A.eX,A.eC,A.dp,A.ds,A.dG,A.e5,A.dw,A.e0,A.dL,A.fe,A.ff,A.fg,A.fh,A.fi,A.fm,A.fj,A.fk])
r(A.bv,A.ad)
q(A.cM,[A.cL,A.aL])
r(A.av,A.bp)
r(A.aQ,A.aR)
q(A.bs,[A.cv,A.I])
q(A.I,[A.bM,A.bO])
r(A.bN,A.bM)
r(A.bq,A.bN)
r(A.bP,A.bO)
r(A.br,A.bP)
q(A.bq,[A.cw,A.cx])
q(A.br,[A.cy,A.cz,A.cA,A.cB,A.cC,A.bt,A.bu])
r(A.bU,A.d_)
q(A.cf,[A.en,A.eo,A.eK,A.et,A.ex,A.ew,A.ev,A.eu,A.eA,A.ez,A.ey,A.dB,A.eH,A.eW,A.eP,A.eO,A.ek,A.f9])
r(A.bJ,A.cX)
r(A.d1,A.c1)
r(A.cc,A.ch)
q(A.ck,[A.dg,A.dH])
q(A.a3,[A.bx,A.cn])
r(A.cY,A.c_)
q(A.eq,[A.aK,A.am,A.Z,A.a7])
q(A.ay,[A.W,A.bd,A.aV,A.a_])
q(A.au,[A.aM,A.dr,A.cJ,A.e1])
q(A.bo,[A.dj,A.dq,A.dx,A.cT])
r(A.bK,A.bB)
r(A.cZ,A.bK)
r(A.cd,A.cl)
s(A.bM,A.x)
s(A.bN,A.L)
s(A.bO,A.x)
s(A.bP,A.L)
s(A.b0,A.bZ)})()
var v={G:typeof self!="undefined"?self:globalThis,typeUniverse:{eC:new Map(),tR:{},eT:{},tPV:{},sEA:[]},mangledGlobalNames:{b:"int",a:"double",ah:"num",D:"String",q:"bool",v:"Null",k:"List",p:"Object",a9:"Map",z:"JSObject"},mangledNames:{},types:["~()","~(z)","~(bI,aU)","v(a,a,a,a)","~(a)","~(@)","~(~())","v()","v(@)","v(a,a,a,a,a,a)","a(a,a,a,a)","~(a,a,a,a,a,b)","~(a,a,a,b)","@()","~(aY,@)","0&(D,b?)","~(D,@)","~(k<a>,b)","~(a,a,a,a,k<a>,a,b)","~(p?,p?)","v(p,an)","~(a,a,a,a,b)","~(a,a,a,a)","~(a,a,D,b)","k<e>(k<a>)","~(D,a4)","a(a,a,a,a,a,a,a)","a(a,a)","~(b,@)","a(a,a,a,a,a,a)","v(a,a,a)","v(a,a,a,a,a,a,a,a)","v(@,an)","@(@,D)","v(a,a,a,a,a)","v(a)","a(a,p?,a)","0&(a)","@(D)","@(@)","q(ab)","ab()","v(~())","~(a,a{horizontal!q})","aM(V,V)","V(a)","e()","~(D{down!q})","q(b,b)"],interceptorsByTag:null,leafTags:null,arrayRti:Symbol("$ti"),rttc:{"2;":(a,b)=>c=>c instanceof A.N&&a.b(c.a)&&b.b(c.b),"17;categoryBits,customColor,density,enableContactEvents,enableHitEvents,enablePreSolveEvents,enableSensorEvents,friction,groupIndex,invokeContactCreation,isSensor,maskBits,restitution,rollingResistance,tangentSpeed,updateBodyMass,userMaterialId":a=>b=>b instanceof A.bR&&A.i_(a,b.a),"23;drawBodyNames,drawBounds,drawCircle,drawContactFeatures,drawContactImpulses,drawContactNormals,drawContacts,drawFrictionImpulses,drawGraphColors,drawIslands,drawJointExtras,drawJoints,drawMass,drawPoint,drawPolygon,drawSegment,drawShapes,drawSolidCapsule,drawSolidCircle,drawSolidPolygon,drawString,drawTransform,drawingBounds":a=>b=>b instanceof A.bS&&A.i_(a,b.a)}}
A.jn(v.typeUniverse,JSON.parse('{"a4":"aj","cF":"aj","bC":"aj","kR":"aR","cq":{"q":[],"o":[]},"bj":{"o":[]},"bm":{"z":[]},"aj":{"z":[]},"n":{"k":["1"],"z":[],"w":["1"]},"cp":{"bz":[]},"dk":{"n":["1"],"k":["1"],"z":[],"w":["1"]},"bk":{"a":[],"ah":[]},"bi":{"a":[],"b":[],"ah":[],"o":[]},"cs":{"a":[],"ah":[],"o":[]},"aO":{"D":[],"h3":[],"o":[]},"aw":{"u":[]},"ao":{"aY":[]},"bR":{"aC":[],"a5":[]},"N":{"b_":[],"a5":[]},"bS":{"aC":[],"a5":[]},"bf":{"bD":["1","2"],"b0":["1","2"],"aP":["1","2"],"bZ":["1","2"],"a9":["1","2"]},"be":{"a9":["1","2"]},"bg":{"be":["1","2"],"a9":["1","2"]},"cr":{"fY":[]},"bv":{"ad":[],"u":[]},"ct":{"u":[]},"cO":{"u":[]},"bT":{"an":[]},"ai":{"as":[]},"cf":{"as":[]},"cg":{"as":[]},"cM":{"as":[]},"cL":{"as":[]},"aL":{"as":[]},"cK":{"u":[]},"av":{"bp":["1","2"],"a9":["1","2"]},"b_":{"a5":[]},"aC":{"a5":[]},"aQ":{"z":[],"o":[]},"aR":{"z":[],"o":[]},"bs":{"z":[]},"cv":{"z":[],"o":[]},"I":{"O":["1"],"z":[]},"bq":{"x":["a"],"I":["a"],"k":["a"],"O":["a"],"z":[],"w":["a"],"L":["a"]},"br":{"x":["b"],"I":["b"],"k":["b"],"O":["b"],"z":[],"w":["b"],"L":["b"]},"cw":{"fq":[],"x":["a"],"I":["a"],"k":["a"],"O":["a"],"z":[],"w":["a"],"L":["a"],"o":[],"x.E":"a"},"cx":{"x":["a"],"I":["a"],"k":["a"],"O":["a"],"z":[],"w":["a"],"L":["a"],"o":[],"x.E":"a"},"cy":{"x":["b"],"I":["b"],"k":["b"],"O":["b"],"z":[],"w":["b"],"L":["b"],"o":[],"x.E":"b"},"cz":{"x":["b"],"I":["b"],"k":["b"],"O":["b"],"z":[],"w":["b"],"L":["b"],"o":[],"x.E":"b"},"cA":{"x":["b"],"I":["b"],"k":["b"],"O":["b"],"z":[],"w":["b"],"L":["b"],"o":[],"x.E":"b"},"cB":{"x":["b"],"I":["b"],"k":["b"],"O":["b"],"z":[],"w":["b"],"L":["b"],"o":[],"x.E":"b"},"cC":{"x":["b"],"I":["b"],"k":["b"],"O":["b"],"z":[],"w":["b"],"L":["b"],"o":[],"x.E":"b"},"bt":{"x":["b"],"I":["b"],"k":["b"],"O":["b"],"z":[],"w":["b"],"L":["b"],"o":[],"x.E":"b"},"bu":{"fu":[],"x":["b"],"I":["b"],"k":["b"],"O":["b"],"z":[],"w":["b"],"L":["b"],"o":[],"x.E":"b"},"d_":{"u":[]},"bU":{"ad":[],"u":[]},"U":{"u":[]},"bJ":{"cX":["1"]},"E":{"at":["1"]},"c1":{"hh":[]},"d1":{"c1":[],"hh":[]},"bp":{"a9":["1","2"]},"aP":{"a9":["1","2"]},"bD":{"b0":["1","2"],"aP":["1","2"],"bZ":["1","2"],"a9":["1","2"]},"cc":{"ch":["k<b>","D"]},"a":{"ah":[]},"b":{"ah":[]},"k":{"w":["1"]},"D":{"h3":[]},"ca":{"u":[]},"ad":{"u":[]},"a3":{"u":[]},"bx":{"u":[]},"cn":{"u":[]},"cD":{"u":[]},"bE":{"u":[]},"cN":{"u":[]},"aX":{"u":[]},"ci":{"u":[]},"cE":{"u":[]},"bA":{"u":[]},"d4":{"an":[]},"J":{"iW":[]},"c_":{"bG":[]},"d2":{"bG":[]},"cY":{"bG":[]},"W":{"ay":[]},"bd":{"ay":[]},"aV":{"ay":[]},"a_":{"ay":[]},"aM":{"au":[]},"dr":{"au":[]},"cJ":{"au":[]},"e1":{"au":[]},"bK":{"bB":["1"]},"cZ":{"bK":["1"],"bB":["1"]},"bL":{"iU":["1"]},"cd":{"cl":[]},"iD":{"k":["b"],"w":["b"]},"fu":{"k":["b"],"w":["b"]},"j_":{"k":["b"],"w":["b"]},"iB":{"k":["b"],"w":["b"]},"iY":{"k":["b"],"w":["b"]},"iC":{"k":["b"],"w":["b"]},"iZ":{"k":["b"],"w":["b"]},"fq":{"k":["a"],"w":["a"]},"iz":{"k":["a"],"w":["a"]}}'))
A.jm(v.typeUniverse,JSON.parse('{"I":1,"ck":2}'))
var u={f:"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\u03f6\x00\u0404\u03f4 \u03f4\u03f6\u01f6\u01f6\u03f6\u03fc\u01f4\u03ff\u03ff\u0584\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u05d4\u01f4\x00\u01f4\x00\u0504\u05c4\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u0400\x00\u0400\u0200\u03f7\u0200\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u03ff\u0200\u0200\u0200\u03f7\x00",e:"Cannot create a joint while the world is stepping. Collision callbacks run inside step(); create after step() returns instead. Destroy operations are deferred automatically.",c:"Error handler must accept one Object or one Object and a StackTrace as arguments, and return a value of the returned future's type",j:"forge2d is not initialized. On the web you must call and await initializeForge2D() before creating a World."}
var t=(function rtii(){var s=A.aG
return{n:s("U"),k:s("bf<aY,@>"),C:s("u"),Z:s("as"),A:s("fY"),U:s("w<b>"),e:s("n<V>"),f:s("n<p>"),w:s("n<aW>"),s:s("n<D>"),J:s("n<H>"),D:s("n<bG>"),F:s("n<e>"),V:s("n<cT>"),u:s("n<a>"),b:s("n<@>"),t:s("n<b>"),a:s("n<ah>"),x:s("n<~()>"),T:s("bj"),m:s("z"),g:s("a4"),E:s("O<@>"),bV:s("av<aY,@>"),Y:s("k<e>"),o:s("k<a>"),j:s("k<@>"),L:s("k<b>"),r:s("k<ah>"),h:s("aQ"),P:s("v"),K:s("p"),cY:s("kS"),cD:s("+()"),G:s("+drawBodyNames,drawBounds,drawCircle,drawContactFeatures,drawContactImpulses,drawContactNormals,drawContacts,drawFrictionImpulses,drawGraphColors,drawIslands,drawJointExtras,drawJoints,drawMass,drawPoint,drawPolygon,drawSegment,drawShapes,drawSolidCapsule,drawSolidCircle,drawSolidPolygon,drawString,drawTransform,drawingBounds(q,q,~(a,a,a,b),q,q,q,q,q,q,q,q,q,q,~(a,a,a,b),~(k<a>,b),~(a,a,a,a,b),q,~(a,a,a,a,a,b),~(a,a,a,a,a,b),~(a,a,a,a,k<a>,a,b),~(a,a,D,b),~(a,a,a,a),+(a,a,a,a)?)"),bQ:s("+(b,b)"),az:s("ab"),l:s("an"),N:s("D"),cm:s("aY"),bW:s("o"),b7:s("ad"),cr:s("bC"),R:s("bG"),b4:s("e"),B:s("cS"),bU:s("cZ<z>"),c:s("E<@>"),aQ:s("E<b>"),y:s("q"),bG:s("q(p)"),p:s("q(b,b,b,b)"),_:s("q(b,b,b,b,a,a)"),i:s("a"),z:s("@"),O:s("@()"),v:s("@(p)"),Q:s("@(p,an)"),S:s("b"),bc:s("at<v>?"),b1:s("z?"),X:s("p?"),aD:s("D?"),I:s("e()?"),c8:s("cR?"),d:s("az<@,@>?"),cG:s("q?"),d3:s("q(b,b)?"),dd:s("a?"),a3:s("b?"),ae:s("ah?"),bp:s("~()?"),aL:s("~(D{down!q})?"),W:s("~(a)?"),q:s("ah"),H:s("~"),M:s("~()")}})();(function constants(){var s=hunkHelpers.makeConstList
B.P=J.co.prototype
B.c=J.n.prototype
B.b=J.bi.prototype
B.f=J.bk.prototype
B.a=J.aO.prototype
B.Q=J.a4.prototype
B.R=J.bm.prototype
B.y=A.bu.prototype
B.z=J.cF.prototype
B.n=J.bC.prototype
B.j=new A.aK(0,"static")
B.o=new A.aK(1,"kinematic")
B.e=new A.aK(2,"dynamic")
B.ak=new A.dg()
B.A=new A.cc()
B.p=function getTagFallback(o) {
  var s = Object.prototype.toString.call(o);
  return s.substring(8, s.length - 1);
}
B.B=function() {
  var toStringFunction = Object.prototype.toString;
  function getTag(o) {
    var s = toStringFunction.call(o);
    return s.substring(8, s.length - 1);
  }
  function getUnknownTag(object, tag) {
    if (/^HTML[A-Z].*Element$/.test(tag)) {
      var name = toStringFunction.call(object);
      if (name == "[object Object]") return null;
      return "HTMLElement";
    }
  }
  function getUnknownTagGenericBrowser(object, tag) {
    if (object instanceof HTMLElement) return "HTMLElement";
    return getUnknownTag(object, tag);
  }
  function prototypeForTag(tag) {
    if (typeof window == "undefined") return null;
    if (typeof window[tag] == "undefined") return null;
    var constructor = window[tag];
    if (typeof constructor != "function") return null;
    return constructor.prototype;
  }
  function discriminator(tag) { return null; }
  var isBrowser = typeof HTMLElement == "function";
  return {
    getTag: getTag,
    getUnknownTag: isBrowser ? getUnknownTagGenericBrowser : getUnknownTag,
    prototypeForTag: prototypeForTag,
    discriminator: discriminator };
}
B.G=function(getTagFallback) {
  return function(hooks) {
    if (typeof navigator != "object") return hooks;
    var userAgent = navigator.userAgent;
    if (typeof userAgent != "string") return hooks;
    if (userAgent.indexOf("DumpRenderTree") >= 0) return hooks;
    if (userAgent.indexOf("Chrome") >= 0) {
      function confirm(p) {
        return typeof window == "object" && window[p] && window[p].name == p;
      }
      if (confirm("Window") && confirm("HTMLElement")) return hooks;
    }
    hooks.getTag = getTagFallback;
  };
}
B.C=function(hooks) {
  if (typeof dartExperimentalFixupGetTag != "function") return hooks;
  hooks.getTag = dartExperimentalFixupGetTag(hooks.getTag);
}
B.F=function(hooks) {
  if (typeof navigator != "object") return hooks;
  var userAgent = navigator.userAgent;
  if (typeof userAgent != "string") return hooks;
  if (userAgent.indexOf("Firefox") == -1) return hooks;
  var getTag = hooks.getTag;
  var quickMap = {
    "BeforeUnloadEvent": "Event",
    "DataTransfer": "Clipboard",
    "GeoGeolocation": "Geolocation",
    "Location": "!Location",
    "WorkerMessageEvent": "MessageEvent",
    "XMLDocument": "!Document"};
  function getTagFirefox(o) {
    var tag = getTag(o);
    return quickMap[tag] || tag;
  }
  hooks.getTag = getTagFirefox;
}
B.E=function(hooks) {
  if (typeof navigator != "object") return hooks;
  var userAgent = navigator.userAgent;
  if (typeof userAgent != "string") return hooks;
  if (userAgent.indexOf("Trident/") == -1) return hooks;
  var getTag = hooks.getTag;
  var quickMap = {
    "BeforeUnloadEvent": "Event",
    "DataTransfer": "Clipboard",
    "HTMLDDElement": "HTMLElement",
    "HTMLDTElement": "HTMLElement",
    "HTMLPhraseElement": "HTMLElement",
    "Position": "Geoposition"
  };
  function getTagIE(o) {
    var tag = getTag(o);
    var newTag = quickMap[tag];
    if (newTag) return newTag;
    if (tag == "Object") {
      if (window.DataView && (o instanceof window.DataView)) return "DataView";
    }
    return tag;
  }
  function prototypeForTagIE(tag) {
    var constructor = window[tag];
    if (constructor == null) return null;
    return constructor.prototype;
  }
  hooks.getTag = getTagIE;
  hooks.prototypeForTag = prototypeForTagIE;
}
B.D=function(hooks) {
  var getTag = hooks.getTag;
  var prototypeForTag = hooks.prototypeForTag;
  function getTagFixed(o) {
    var tag = getTag(o);
    if (tag == "Document") {
      if (!!o.xmlVersion) return "!Document";
      return "!HTMLDocument";
    }
    return tag;
  }
  function prototypeForTagFixed(tag) {
    if (tag == "Document") return null;
    return prototypeForTag(tag);
  }
  hooks.getTag = getTagFixed;
  hooks.prototypeForTag = prototypeForTagFixed;
}
B.q=function(hooks) { return hooks; }

B.H=new A.cE()
B.h=new A.dy()
B.m=new A.eD()
B.r=new A.eG()
B.i=new A.d1()
B.l=new A.d4()
B.I=new A.a7(0,"keyDown")
B.J=new A.a7(1,"keyUp")
B.K=new A.a7(2,"pointerDown")
B.L=new A.a7(3,"pointerMove")
B.M=new A.a7(4,"pointerUp")
B.N=new A.a7(5,"pointerCancel")
B.O=new A.a7(6,"click")
B.S=new A.Z(0,"distance")
B.T=new A.Z(1,"filter")
B.U=new A.Z(2,"motor")
B.V=new A.Z(3,"mouse")
B.W=new A.Z(4,"prismatic")
B.X=new A.Z(5,"revolute")
B.Y=new A.Z(6,"weld")
B.Z=new A.Z(7,"wheel")
B.t=s([B.S,B.T,B.U,B.V,B.W,B.X,B.Y,B.Z],A.aG("n<Z>"))
B.a1=new A.am(0,"circle")
B.a2=new A.am(1,"capsule")
B.a3=new A.am(2,"segment")
B.a4=new A.am(3,"polygon")
B.a5=new A.am(4,"chainSegment")
B.u=s([B.a1,B.a2,B.a3,B.a4,B.a5],A.aG("n<am>"))
B.v=s([],t.b)
B.k=s([8166655,3718648,16498468,16478597,3462041,10980346],t.t)
B.w=s([B.j,B.o,B.e],A.aG("n<aK>"))
B.a_={}
B.x=new A.bg(B.a_,[],A.aG("bg<aY,@>"))
B.a0=new A.N(4294967295,4294967295)
B.d=new A.al(1,0)
B.a6=new A.ao("call")
B.a7=A.a2("kN")
B.a8=A.a2("kO")
B.a9=A.a2("fq")
B.aa=A.a2("iz")
B.ab=A.a2("iB")
B.ac=A.a2("iC")
B.ad=A.a2("iD")
B.ae=A.a2("p")
B.af=A.a2("iY")
B.ag=A.a2("iZ")
B.ah=A.a2("j_")
B.ai=A.a2("fu")
B.aj=new A.dH(!1)})();(function staticFields(){$.eE=null
$.af=A.h([],t.f)
$.h4=null
$.fU=null
$.fT=null
$.hX=null
$.hR=null
$.i1=null
$.eY=null
$.f4=null
$.fK=null
$.eF=A.h([],A.aG("n<k<p>?>"))
$.b6=null
$.c5=null
$.c6=null
$.fD=!1
$.y=B.i
$.he=""
$.hf=null
$.a6=null
$.d=null})();(function lazyInitializers(){var s=hunkHelpers.lazyFinal
s($,"kQ","i6",()=>A.hW("_$dart_dartClosure"))
s($,"kP","bb",()=>A.hW("_$dart_dartClosure_dartJSInterop"))
s($,"l9","im",()=>A.h([new J.cp()],A.aG("n<bz>")))
s($,"kU","i7",()=>A.ae(A.dE({
toString:function(){return"$receiver$"}})))
s($,"kV","i8",()=>A.ae(A.dE({$method$:null,
toString:function(){return"$receiver$"}})))
s($,"kW","i9",()=>A.ae(A.dE(null)))
s($,"kX","ia",()=>A.ae(function(){var $argumentsExpr$="$arguments$"
try{null.$method$($argumentsExpr$)}catch(r){return r.message}}()))
s($,"l_","id",()=>A.ae(A.dE(void 0)))
s($,"l0","ie",()=>A.ae(function(){var $argumentsExpr$="$arguments$"
try{(void 0).$method$($argumentsExpr$)}catch(r){return r.message}}()))
s($,"kZ","ic",()=>A.ae(A.hb(null)))
s($,"kY","ib",()=>A.ae(function(){try{null.$method$}catch(r){return r.message}}()))
s($,"l2","ih",()=>A.ae(A.hb(void 0)))
s($,"l1","ig",()=>A.ae(function(){try{(void 0).$method$}catch(r){return r.message}}()))
s($,"l3","fN",()=>A.j4())
s($,"l7","il",()=>A.iL(4096))
s($,"l5","ij",()=>new A.eP().$0())
s($,"l6","ik",()=>new A.eO().$0())
s($,"l4","ii",()=>new Int8Array(A.jU(A.h([-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-1,-2,-2,-2,-2,-2,62,-2,62,-2,63,52,53,54,55,56,57,58,59,60,61,-2,-2,-2,-1,-2,-2,-2,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,-2,-2,-2,-2,63,-2,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,-2,-2,-2,-2,-2],t.t))))
s($,"l8","dd",()=>A.hZ(B.ae))
s($,"lc","de",()=>A.h([A.aT(new A.fe(),10,"Drag boxes with the pointer","Pyramid",26),A.aT(new A.ff(),17,"A heavy ball topples the tower; drag the debris around","Domino tower",46),A.aT(new A.fg(),0,"A spinning paddle keeps the balls moving","Ball cage",34),A.aT(new A.fh(),12,"Hundreds of balls, one container","Circle stress",34),A.aT(new A.fi(),8,"A soft body of springs; drag it around","Blob",26),A.aT(new A.fj(),6,"A plank bridge under load","Bridge",24),A.aT(new A.fk(),3,"Drive with the left and right arrow keys","Racer",24)],A.aG("n<ab>")))})();(function nativeSupport(){!function(){var s=function(a){var m={}
m[a]=1
return Object.keys(hunkHelpers.convertToFastObject(m))[0]}
v.getIsolateTag=function(a){return s("___dart_"+a+v.isolateTag)}
var r="___dart_isolate_tags_"
var q=Object[r]||(Object[r]=Object.create(null))
var p="_ZxYxX"
for(var o=0;;o++){var n=s(p+"_"+o+"_")
if(!(n in q)){q[n]=1
v.isolateTag=n
break}}v.dispatchPropertyName=v.getIsolateTag("dispatch_record")}()
hunkHelpers.setOrUpdateInterceptorsByTag({SharedArrayBuffer:A.aR,ArrayBuffer:A.aQ,ArrayBufferView:A.bs,DataView:A.cv,Float32Array:A.cw,Float64Array:A.cx,Int16Array:A.cy,Int32Array:A.cz,Int8Array:A.cA,Uint16Array:A.cB,Uint32Array:A.cC,Uint8ClampedArray:A.bt,CanvasPixelArray:A.bt,Uint8Array:A.bu})
hunkHelpers.setOrUpdateLeafTags({SharedArrayBuffer:true,ArrayBuffer:true,ArrayBufferView:false,DataView:true,Float32Array:true,Float64Array:true,Int16Array:true,Int32Array:true,Int8Array:true,Uint16Array:true,Uint32Array:true,Uint8ClampedArray:true,CanvasPixelArray:true,Uint8Array:false})
A.I.$nativeSuperclassTag="ArrayBufferView"
A.bM.$nativeSuperclassTag="ArrayBufferView"
A.bN.$nativeSuperclassTag="ArrayBufferView"
A.bq.$nativeSuperclassTag="ArrayBufferView"
A.bO.$nativeSuperclassTag="ArrayBufferView"
A.bP.$nativeSuperclassTag="ArrayBufferView"
A.br.$nativeSuperclassTag="ArrayBufferView"})()
Function.prototype.$0=function(){return this()}
Function.prototype.$1=function(a){return this(a)}
Function.prototype.$2=function(a,b){return this(a,b)}
Function.prototype.$3=function(a,b,c){return this(a,b,c)}
Function.prototype.$4=function(a,b,c,d){return this(a,b,c,d)}
Function.prototype.$5=function(a,b,c,d,e){return this(a,b,c,d,e)}
Function.prototype.$6=function(a,b,c,d,e,f){return this(a,b,c,d,e,f)}
Function.prototype.$7=function(a,b,c,d,e,f,g){return this(a,b,c,d,e,f,g)}
convertAllToFastObject(w)
convertToFastObject($);(function(a){if(typeof document==="undefined"){a(null)
return}if(typeof document.currentScript!="undefined"){a(document.currentScript)
return}var s=document.scripts
function onLoad(b){for(var q=0;q<s.length;++q){s[q].removeEventListener("load",onLoad,false)}a(b.target)}for(var r=0;r<s.length;++r){s[r].addEventListener("load",onLoad,false)}})(function(a){v.currentScript=a
var s=A.f5
if(typeof dartMainRunner==="function"){dartMainRunner(s,[])}else{s([])}})})()
//# sourceMappingURL=main.dart.js.map
