import React from 'react';

function CVComponent(props) {
    return (
        <div>
            <img src={props.src} style={{width:"100%",height:"300px"}}/>
        </div>
    );
}

export default CVComponent;