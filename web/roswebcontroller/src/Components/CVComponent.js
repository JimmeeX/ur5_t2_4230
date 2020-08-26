import React from 'react';

function CVComponent(props) {
    return (
        <div>
            <img src={props.src} style={{width:"320px",height:"240px"}}/>
        </div>
    );
}

export default CVComponent;