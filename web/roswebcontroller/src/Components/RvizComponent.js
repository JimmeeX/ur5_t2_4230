import React from 'react';

function RvizComponent(props) {
    return (
        <div>
            <img style={{width:"100%",height:"300px"}} src={props.src}/>
        </div>
    );
}

export default RvizComponent;