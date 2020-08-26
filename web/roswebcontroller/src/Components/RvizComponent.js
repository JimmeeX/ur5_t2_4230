import React from 'react';

function RvizComponent(props) {
    return (
        <div>
            <img style={{width:"320px",height:"240px"}} src={props.src}/>
        </div>
    );
}

export default RvizComponent;